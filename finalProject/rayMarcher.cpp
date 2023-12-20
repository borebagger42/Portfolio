#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#define cimg_use_png
#include "CImg.h"

#define EPSILON 1e-6

// Global Variables:
glm::vec3 globalCameraPosition;
glm::vec3 globalCameraTarget;
glm::vec3 globalCameraUp;
glm::vec3 globalRayDirection;
float parentRotationAngle = 0.0f;
int globalWidth;
int globalHeight;

using namespace cimg_library;

// This program is a Ray Marching algorithm
// written in C++ using CImg and glm. This
// version does not include shading
// calculations, but does include lighting
// calculations for diffuse and specular
// lighting. The command line argument
// used to compile the code was as follows:

// g++ -o rayMarcher rayMarcher.cpp -lpng -lpthread -lX11 -lm

// Parts of this code are recycled from
// programs written for other assignments
// in this class. This has led to some
// vestigial code (such as the transform
// matrices and parents/children) being
// present without any clear purpose.

// These structs are used to enable spheres,
// triangles, boxes, and cylinders:

struct Sphere {
    glm::vec3 center;
    float radius;
    glm::vec3 color;
};

struct Triangle {
    glm::vec3 vertex1;
    glm::vec3 vertex2;
    glm::vec3 vertex3;
    glm::vec3 color;
};

struct Box {
    glm::vec3 center;
    float size;
    glm::vec3 color;
};

struct Cylinder {
    glm::vec3 center;
    float rad;
    float h;
    glm::vec3 color;
};

// This is the overall Shape struct, which
// can be used for any of the four shapes,
// and includes a number of setters/getters.

struct Shape {
    enum Type { SPHERE, TRIANGLE, BOX, CYLINDER };
    Type type;
    union {
        Sphere sphere;
        Triangle triangle;
        Box box;
        Cylinder cylinder;
    };
    glm::vec3 color;
  glm::mat4 transform; // Vestigial Code
  std::vector<int> children; // Vestigial Code
    Shape(Type t) : type(t), transform(glm::mat4(1.0f)) {}

    void setSphere(const Sphere& s) {
        sphere = s;
        color = s.color;
    }

    void setTriangle(const Triangle& t) {
        triangle = t;
        color = t.color;
    }

    void setBox(const Box& b) {
        box = b;
        color = b.color;
    }

    void setCylinder(const Cylinder& c) {
        cylinder = c;
        color = c.color;
    }

   // Vestigial Code
    void applyTransform(const glm::mat4& newTransform) {
        transform = newTransform;
    }

    // Vestigial Code
    glm::mat4 getTransform() const {
        return transform;
    }
};

// These are the signed distance functions
// for each shape type. They are used to
// calculate the distance to the surface
// of an object from a certain point in
// space, and are necessary for ray
// marching programs.

float signedDistanceSphere(const glm::vec3& rayOrigin, const Sphere& sphere) {
    return length(rayOrigin - sphere.center) - sphere.radius;
}

float signedDistanceTriangle(const glm::vec3& rayOrigin, const Triangle& triangle) {
    glm::vec3 v21 = triangle.vertex2 - triangle.vertex1;
    glm::vec3 p1 = rayOrigin - triangle.vertex1;
    glm::vec3 v32 = triangle.vertex3 - triangle.vertex2;
    glm::vec3 p2 = rayOrigin - triangle.vertex2;
    glm::vec3 v13 = triangle.vertex1 - triangle.vertex3;
    glm::vec3 p3 = rayOrigin - triangle.vertex3;
    glm::vec3 nor = glm::cross(v21, v13);

    return glm::sqrt((glm::sign(glm::dot(glm::cross(v21, nor), p1)) +
        glm::sign(glm::dot(glm::cross(v32, nor), p2)) +
        glm::sign(glm::dot(glm::cross(v13, nor), p3)) < 2.0)
        ? glm::min(glm::min(
            glm::dot(v21 * glm::clamp(glm::dot(v21, p1) / glm::dot(v21, v21), 0.0f, 1.0f) - p1,
                v21 * glm::clamp(glm::dot(v21, p1) / glm::dot(v21, v21), 0.0f, 1.0f) - p1),
            glm::dot(v32 * glm::clamp(glm::dot(v32, p2) / glm::dot(v32, v32), 0.0f, 1.0f) - p2,
                v32 * glm::clamp(glm::dot(v32, p2) / glm::dot(v32, v32), 0.0f, 1.0f) - p2)),
            glm::dot(v13 * glm::clamp(glm::dot(v13, p3) / glm::dot(v13, v13), 0.0f, 1.0f) - p3,
                v13 * glm::clamp(glm::dot(v13, p3) / glm::dot(v13, v13), 0.0f, 1.0f) - p3))
        : glm::dot(nor, p1) * glm::dot(nor, p1) / glm::dot(nor, nor));
}

float signedDistanceBox(const glm::vec3 rayOrigin, const Box& box) {
    glm::vec3 q = glm::abs(rayOrigin - box.center) - 0.5f * box.size;
    return glm::length(glm::max(q, glm::vec3(0.0f))) + glm::min(glm::max(q.x, glm::max(q.y, q.z)), 0.0f);
}

float signedDistanceCylinder(const glm::vec3 rayOrigin, const Cylinder& cylinder) {
    glm::vec2 d = glm::abs(glm::vec2(glm::length(glm::vec2(rayOrigin.x, rayOrigin.z) - glm::vec2(cylinder.center.x, cylinder.center.z)), rayOrigin.y - cylinder.center.y)) - glm::vec2(cylinder.rad, cylinder.h * 0.5f);
    float distanceToSide = glm::length(glm::max(d, 0.0f));
    float distanceToTopBottom = glm::min(glm::max(d.x, d.y), 0.0f);
    return distanceToSide + distanceToTopBottom;
}

// The readSetupFile function takes in a
// file from the user and crafts a scene
// from its specifications. It goes down
// every line in the file and performs
// certain functions based on the given
// commands.

void readSetupFile(const std::string& filename, std::vector<Shape>& scene) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open the setup file.\n";
        return;
    }

    std::string line;
    std::map<std::string, int> objectIndices;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string command;
        iss >> command;

        if (command == "image") {
            iss >> globalWidth >> globalHeight;
        }
        else if (command == "camera_position") {
            glm::vec3 position;
            iss >> position.x >> position.y >> position.z;
            globalCameraPosition = position;
        }
        else if (command == "camera_target") {
            glm::vec3 target;
            iss >> target.x >> target.y >> target.z;
            globalCameraTarget = target;
        }
        else if (command == "camera_up") {
            glm::vec3 up;
            iss >> up.x >> up.y >> up.z;
            globalCameraUp = up;
        }
        else if (command == "sphere") {
            Sphere sphere;
            iss >> sphere.center.x >> sphere.center.y >> sphere.center.z >> sphere.radius >> sphere.color.r >> sphere.color.g >> sphere.color.b;
            Shape shape(Shape::SPHERE);
            shape.setSphere(sphere);
            shape.applyTransform(glm::mat4(1.0f));
            scene.emplace_back(shape);
        }
        else if (command == "triangle") {
            Triangle triangle;
            iss >> triangle.vertex1.x >> triangle.vertex1.y >> triangle.vertex1.z
                >> triangle.vertex2.x >> triangle.vertex2.y >> triangle.vertex2.z
                >> triangle.vertex3.x >> triangle.vertex3.y >> triangle.vertex3.z
                >> triangle.color.r >> triangle.color.g >> triangle.color.b;
            Shape shape(Shape::TRIANGLE);
            shape.setTriangle(triangle);
            shape.applyTransform(glm::mat4(1.0f));
            scene.emplace_back(shape);
        }
        else if (command == "box") {
            Box box;
            iss >> box.center.x >> box.center.y >> box.center.z >> box.size >> box.color.r >> box.color.g >> box.color.b;
            Shape shape(Shape::BOX);
            shape.setBox(box);
            shape.applyTransform(glm::mat4(1.0f));
            scene.emplace_back(shape);
        }
        else if (command == "cylinder") {
            Cylinder cylinder;
            iss >> cylinder.center.x >> cylinder.center.y >> cylinder.center.z >> cylinder.rad >> cylinder.h >> cylinder.color.r >> cylinder.color.g >> cylinder.color.b;
            Shape shape(Shape::CYLINDER);
            shape.setCylinder(cylinder);
            shape.applyTransform(glm::mat4(1.0f));
            scene.emplace_back(shape);
        }
        else if (command == "name") {
            std::string objectName;
            iss >> objectName;
            objectIndices[objectName] = static_cast<int>(scene.size()) - 1;
        }
        else if (command == "parent") { // Vestigial Code
            std::string parentName;
            iss >> parentName;
            int childIndex = static_cast<int>(scene.size()) - 1;

            if (objectIndices.find(parentName) != objectIndices.end()) {
                int parentIndex = objectIndices[parentName];
                glm::mat4 currentTransform = scene[childIndex].getTransform();
                scene[childIndex].applyTransform(currentTransform);
                scene[parentIndex].children.push_back(childIndex);
            }
        }
        else if (command == "transform") { // Vestigial Code
            glm::mat4 transformMatrix = glm::mat4(1.0f);

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    iss >> transformMatrix[j][i];
                }
            }

            scene.back().applyTransform(transformMatrix);
        }
    }
}

// This is the main function of this
// program, which includes the ray
// marching and lighting code. It
// takes no parameters from the user,
// but instead calls in a scene
// description file with instructions
// for how the scene is laid out.

int main() {
    float delta = .001;
    int maxIterations = 100;
    int maxDistance = 100;
    std::vector<Shape> originalScene;
    std::vector<Shape> scene;
    readSetupFile("scene.txt", scene);

    const int width = globalWidth;
    const int height = globalHeight;

    CImg<unsigned char> image(width, height, 1, 3, 0);

    glm::mat4 viewMatrix = glm::lookAt(globalCameraPosition, globalCameraTarget, globalCameraUp);

    float aspectRatio = static_cast<float>(width) / height;

    // Ray Marching Loop
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
	    std::cout << x << " " << y << std::endl;
            float distTraveled = 0;
            float minSignedDistance = 100;
            float signedDist;
            bool hitFound = false;
	    bool colorPicked = false;
            glm::vec3 color = glm::vec3(0.0f, 0.0f, 0.0f);
	    
            float ndcX = aspectRatio * ((2.0f * x) / width - 1.0f);
            float ndcY = 1.0f - (2.0f * y) / height;
            glm::vec4 clipCoords(ndcX, ndcY, -1.0f, 1.0f);
            glm::vec4 eyeCoords = glm::inverse(viewMatrix) * clipCoords;
            glm::vec3 rayDirection = -glm::normalize(glm::vec3(eyeCoords));
            
            glm::vec3 currentCoords;
            int iterations = 0;

            while (iterations < maxIterations && distTraveled < maxDistance && !colorPicked) {
                for (const auto& shape : scene) {
		  hitFound = false;
                    currentCoords = globalCameraPosition + (distTraveled * rayDirection);
                    if (shape.type == Shape::TRIANGLE) {
                        signedDist = signedDistanceTriangle(currentCoords, shape.triangle);
                        if (signedDist < delta) {
                          color = shape.triangle.color;
                          hitFound = true;
                        }
                        else if (signedDist <= minSignedDistance) {
                          minSignedDistance = signedDist;
                        }
                    }
                    else if (shape.type == Shape::SPHERE) {
                        signedDist = signedDistanceSphere(currentCoords, shape.sphere);
                        if (signedDist < delta) {
                          color = shape.sphere.color;
                          hitFound = true;
                        }
                        else if (signedDist <= minSignedDistance) {
                          minSignedDistance = signedDist;
                        }
                    }
                    else if (shape.type == Shape::BOX) {
                        signedDist = signedDistanceBox(currentCoords, shape.box);
                        if (signedDist < delta) {
                          color = shape.box.color;
                          hitFound = true;
                        }
                        else if (signedDist <= minSignedDistance) {
                          minSignedDistance = signedDist;
                        }
                    }
                    else if (shape.type == Shape::CYLINDER) {
                        signedDist = signedDistanceCylinder(currentCoords, shape.cylinder);
                        if (signedDist < delta) {
                          color = shape.cylinder.color;
                          hitFound = true;
                        }
                        else if (signedDist <= minSignedDistance) {
                          minSignedDistance = signedDist;
                        }
                    }

		    // Lighting Calculations
		    if (hitFound) {
		      using namespace glm;
		      vec3 normalVector;
		      vec3 ambientColor = vec3(0.1f, 0.1f, 0.1f);
		      vec3 lightPosition(-5.0f, -5.0f, 5.0f);
		      vec3 lightDirection = normalize(lightPosition - currentCoords);
		      
		      if (shape.type == Shape::SPHERE) {
			normalVector = normalize(currentCoords - shape.sphere.center);
		      }
		      else if (shape.type == Shape::TRIANGLE) {
		        vec3 edge1 = shape.triangle.vertex2 - shape.triangle.vertex1;
			vec3 edge2 = shape.triangle.vertex3 - shape.triangle.vertex1;
			
		        normalVector = -normalize(cross(edge1, edge2));
		      }
		      else if (shape.type == Shape::BOX) {
		        normalVector = normalize(currentCoords - shape.box.center);
			float maxComponent = max(abs(normalVector.x), max(abs(normalVector.y), abs(normalVector.z)));

			if (abs(maxComponent - abs(normalVector.x)) < EPSILON) {
			  normalVector.x = glm::sign(normalVector.x);
			  normalVector.y = normalVector.z = 0.0f;
			} else if (abs(maxComponent - abs(normalVector.y)) < EPSILON) {
			  normalVector.y = glm::sign(normalVector.y);
			  normalVector.x = normalVector.z = 0.0f;
			} else if (abs(maxComponent - abs(normalVector.z)) < EPSILON) {
			  normalVector.z = glm::sign(normalVector.z);
			  normalVector.x = normalVector.y = 0.0f;
			}

		      }
		      else if (shape.type == Shape::CYLINDER) {
		        vec2 d = abs(vec2(length(vec2(currentCoords.x, currentCoords.z) - vec2(shape.cylinder.center.x, shape.cylinder.center.z)), currentCoords.y - shape.cylinder.center.y)) - vec2(shape.cylinder.rad, shape.cylinder.h * 0.5f);

			if (length(d) < EPSILON) {
			  normalVector = normalize(vec3(0.0f, glm::sign(currentCoords.y - shape.cylinder.center.y), 0.0f));
			} else {
			  normalVector = normalize(vec3(currentCoords.x - shape.cylinder.center.x, 0.0f, currentCoords.z - shape.cylinder.center.z));
			}
		      }

		      // Shadow Calculations
		      bool inShadow = false;
		      vec3 shadowRayDirection = normalize(lightPosition - currentCoords);
		      float shadowRayDistance = glm::length(lightPosition - currentCoords);
		      
		      for (float t = delta; t < shadowRayDistance; t+= delta) {
			if (inShadow) break;

			vec3 shadowRayOrigin = currentCoords + t * shadowRayDirection;
			for (const auto& otherShape : scene) {
			  if (&otherShape != &shape) {
			    float shadowDist = 1;
			    if (otherShape.type == Shape::SPHERE) {
			      shadowDist = signedDistanceSphere(shadowRayOrigin, otherShape.sphere);
			    } else if (otherShape.type == Shape::BOX) {
			      shadowDist = signedDistanceBox(shadowRayOrigin, otherShape.box);
			    } else if (otherShape.type == Shape::TRIANGLE) {
			      shadowDist = signedDistanceTriangle(shadowRayOrigin, otherShape.triangle);
			    } else if (otherShape.type == Shape::CYLINDER) {
			      shadowDist = signedDistanceCylinder(shadowRayOrigin, otherShape.cylinder);
			    }
			    if (shadowDist < delta) {
			      inShadow = true;
			      break;
			    }
			  }
			}
		      }

		      // More Lighting Calculations
		      float diffuseIntensity = glm::max(0.0f, dot(normalVector, lightDirection));

		      // Diffuse Lighting Calculations
		      vec3 diffuseColor = shape.color;
		      vec3 lightColor = vec3(1.0f, 1.0f, 1.0f);

		      // Specular Lighting Calculations
		      float shininess = 10.0f;
		      vec3 viewDirection = normalize(globalCameraPosition - currentCoords);
		      vec3 reflectionDirection = reflect(-lightDirection, normalVector);
		      float specularIntensity = pow(max(0.0f, dot(viewDirection, reflectionDirection)), shininess);
		      vec3 specularColor = vec3(0.5f, 0.5f, 0.5f);

		      // Color Formula
		      color = glm::clamp(ambientColor + diffuseIntensity * diffuseColor * lightColor + specularIntensity * specularColor, 0.0f, 1.0f);
		      if (inShadow) color = color * 0.2f;
		      
		      colorPicked = true;
	            }
                }
                distTraveled += minSignedDistance;
                iterations += 1;
            }

            image(x, y, 0, 0) = static_cast<unsigned char>(color.r * 255);
            image(x, y, 0, 1) = static_cast<unsigned char>(color.g * 255);
            image(x, y, 0, 2) = static_cast<unsigned char>(color.b * 255);
        }
    }
    image.display("Ray Marching");

    return 0;
}
