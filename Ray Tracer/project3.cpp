#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#define EPSILON 1e-6

#define cimg_use_png
#include "CImg.h"

// Global Variables:
glm::vec3 globalCameraPosition;
glm::vec3 globalCameraTarget;
glm::vec3 globalCameraUp;
glm::vec3 globalRayDirection;
float parentRotationAngle = 0.0f;
int globalWidth;
int globalHeight;

using namespace cimg_library;

// This program was created and run on an ubuntu-
// based virtual machine. The following line was
// successfully used to compile this code:
//
// g++ -o a project3.cpp -lpng -lpthread -lX11 -lm
//
// Project3.cpp uses CImg and glm to read a scene
// description file, create a list of objects and
// their position/direction/size/etc in a 3D space
// and use ray tracing to display an image of that
// space from a direction chosen by the user.

// !!!!!!!!!!!!!!!!!!IMPORTANT!!!!!!!!!!!!!!!!!!!!!
// Use the arrow keys to move the object left, right,
// forwards, or backwards. Only make one input at a
// time, and be patient because (at least on my
// laptop, it can take almost 10 seconds to complete).
// Use the "Q" and "E" keys to the turn the object
// left or right. All changes are made to the first
// object in the scene file, which is assumed to be
// the "main" object in your scene.

// The scene description file is organized as follows:
// image x y
// camera_position x y z
// camera_target x y z
// camera_up x y z
//
// sphere x y z size r g b
//   OR
// triangle x1 y1 z1 x2 y2 z2 x3 y3 z3 r g b
//   OR
// plane point_x point_y point_z normal_x normal_y normal_z r g b
//
// name object_name
// parent parent_name
// transform 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1
//   OR any other transformation matrix (^ identity matrix)


// printMatrix is used to update the user on the
// location of the main object of the scene, with
// index 0. It is a simple printing statement.
void printMatrix(const glm::mat4& matrix) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      std::cout << matrix[j][i] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

// These structs are used to enable spheres,
// triangles, and planes:
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

struct Plane {
  glm::vec3 point;
  glm::vec3 normal;
  glm::vec3 color;
};

// This is the overall Shape struct, which
// can be used for any of the three shapes,
// and includes a number of setters/getters.
struct Shape {
  enum Type { SPHERE, TRIANGLE, PLANE };
  Type type;
  union {
    Sphere sphere;
    Triangle triangle;
    Plane plane;
  };
  glm::vec3 color;
  glm::mat4 transform;
  std::vector<int> children;
  Shape(Type t) : type(t), transform(glm::mat4(1.0f)) {}

  void setSphere(const Sphere& s) {
    sphere = s;
    color = s.color;
  }

  void setTriangle(const Triangle& t) {
    triangle = t;
    color = t.color;
  }

  void setPlane(const Plane& p) {
    plane = p;
    color = p.color;
  }

  void applyTransform(const glm::mat4& newTransform) {
    transform = newTransform;
  }

  glm::mat4 getTransform() const {
    return transform;
  }
};

// This is the intersection calculator for
// spheres, which finds if a ray hits a
// sphere based on its position and size.
bool intersectSphere(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const Sphere& sphere, float& t, glm::vec3& normal) {
  glm::vec3 rayToSphere = sphere.center - rayOrigin;
  float projection = glm::dot(rayToSphere, rayDirection);
  glm::vec3 closestPoint = rayOrigin + projection * rayDirection;
  float distanceToCenter = glm::length(closestPoint - sphere.center);

  if (distanceToCenter <= sphere.radius) {
    float distanceToIntersection = projection - glm::sqrt(sphere.radius * sphere.radius - distanceToCenter * distanceToCenter);
    
    if (distanceToIntersection >= 0) {
      t = distanceToIntersection;
      normal = glm::normalize(closestPoint - sphere.center);
      return true;
    }
  }
  return false;
}

// This is the intersection calculator for
// triangles. It is a bit more complex and
// uses the triangle's three vertices.
bool intersectTriangle(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const Triangle& triangle, float& t, glm::vec3& normal) {
  glm::vec3 edge1 = triangle.vertex2 - triangle.vertex1;
  glm::vec3 edge2 = triangle.vertex3 - triangle.vertex1;
  glm::vec3 h = glm::cross(rayDirection, edge2);
  float a = glm::dot(edge1, h);

  if (a > -EPSILON && a < EPSILON) {
    return false;
  }

  float f = 1.0f / a;
  glm::vec3 s = rayOrigin - triangle.vertex1;
  float u = f * glm::dot(s, h);

  if (u < 0.0f || u > 1.0f) {
    return false;
  }

  glm::vec3 q = glm::cross(s, edge1);
  float v = f * glm::dot(rayDirection, q);

  if (v < 0.0f || u + v > 1.0f) {
    return false;
  }

  t = f * glm::dot(edge2, q);

  if (t > EPSILON) {
    normal = glm::normalize(glm::cross(edge1, edge2));
    return true;
  }

  return false;
}

// This is the intersection calculator for
// planes. It uses the give point on the
// plane and the plane's normal vector.
bool intersectPlane(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const Plane& plane, float& t, glm::vec3& normal) {
  glm::vec3 w0 = rayOrigin - plane.point;
  float a = -glm::dot(plane.normal, w0);
  float b = glm::dot(rayDirection, plane.normal);

  if (glm::abs(b) < EPSILON) {
    return false;
  }

  t = a / b;
  normal = plane.normal;

  return t >= 0;
}

// This is the basic intersect function,
// which calls a more specific intersect
// function depending on the given shape.
bool intersect(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const Shape& shape, float& t, glm::vec3& normal) {
  if (shape.type == Shape::SPHERE) {
    return intersectSphere(rayOrigin, rayDirection, shape.sphere, t, normal);
  } else if (shape.type == Shape::TRIANGLE) {
    return intersectTriangle(rayOrigin, rayDirection, shape.triangle, t, normal);
  } else if (shape.type == Shape::PLANE) {
    return intersectPlane(rayOrigin, rayDirection, shape.plane, t, normal);
  } else {
    return false;
  }
}

// This is the traceRay function, which
// detects what shapes are hit by any
// given ray. To make sure that the
// frontmost object in a scene is
// properly shown, it goes through every
// shape in the scene for every ray.
glm::vec3 traceRay(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const std::vector<Shape>& shapes) {
  float closestT = std::numeric_limits<float>::infinity();
  glm::vec3 closestNormal;
  glm::vec3 closestColor;

  for (const auto& shape : shapes) {
    glm::mat4 inverseTransform = glm::inverse(shape.getTransform());
    glm::vec3 localRayOrigin = glm::vec3(inverseTransform * glm::vec4(rayOrigin, 1.0f));
    glm::vec3 localRayDirection = glm::vec3(inverseTransform * glm::vec4(rayDirection, 0.0f));
    
    float t;
    glm::vec3 normal;
    if (intersect(localRayOrigin, localRayDirection, shape, t, normal) && t < closestT) {
      closestT = t;
      closestNormal = glm::normalize(glm::vec3(glm::transpose(inverseTransform) * glm::vec4(normal, 0.0f)));
      closestColor = shape.color;
    }
  }

  if (closestT < std::numeric_limits<float>::infinity()) {
    return closestColor;
  }

  return glm::vec3(0.0f, 0.0f, 0.0f);
}

// The applyTranslation function is used to
// move the shape forward/back/left/right.
// The object always moves with respect to
// the camera, regardless of its direction.
void applyTranslation(std::vector<Shape>& scene, int objectIndex, const glm::vec3& translateMatrix) {
  Shape& object = scene[objectIndex];

  glm::mat4 totalTransform = glm::translate(glm::mat4(1.0f), translateMatrix) * object.getTransform();
  object.applyTransform(totalTransform);

  for (int childIndex : object.children) {
    applyTranslation(scene, childIndex, translateMatrix);
  }
}

// The applyRotation function is used to turn
// the object left or right. The object always
// turns in place, regardless of its location.
void applyRotation(std::vector<Shape>& scene, int objectIndex, float angle, const glm::vec3& rotationAxis) {
  Shape& object = scene[objectIndex];
  
  object.applyTransform(glm::rotate(object.getTransform(), glm::radians(angle), rotationAxis));

  for (int childIndex : object.children) {
    applyRotation(scene, childIndex, angle, rotationAxis);
  }
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
    } else if (command == "camera_target") {
      glm::vec3 target;
      iss >> target.x >> target.y >> target.z;
      globalCameraTarget = target;
    } else if (command == "camera_up") {
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
    else if (command == "plane") {
      Plane plane;
      iss >> plane.point.x >> plane.point.y >> plane.point.z
	  >> plane.normal.x >> plane.normal.y >> plane.normal.z
	  >> plane.color.r >> plane.color.g >> plane.color.b;
      Shape shape(Shape::PLANE);
      shape.setPlane(plane);
      shape.applyTransform(glm::mat4(1.0f));
      scene.emplace_back(shape);
    }
    else if (command == "name") {
      std::string objectName;
      iss >> objectName;
      objectIndices[objectName] = static_cast<int>(scene.size()) - 1;
    }
    else if (command == "parent") {
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
    else if (command == "transform") {
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

// The main function, as usual, is where
// everything comes together. It takes
// in a scene file from the user, crafts
// a scene based on the given information,
// and uses raytracing to display it to
// the user. It uses an endless while loop
// to let the user make any number of
// alterations to the shape (translation
// and rotation) until they close the window.
int main() {
  std::vector<Shape> originalScene;
  std::vector<Shape> scene;
  readSetupFile("scene.txt", scene);

  const int width = globalWidth;
  const int height = globalHeight;

  CImg<unsigned char> image(width, height, 1, 3, 0);
  CImgDisplay display(image, "Ray Tracing");

  glm::mat4 viewMatrix = glm::lookAt(globalCameraPosition, globalCameraTarget, globalCameraUp);

  float aspectRatio = static_cast<float>(width) / height;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float ndcX = aspectRatio * ((2.0f * x) / width - 1.0f);
      float ndcY = 1.0f - (2.0f * y) / height;
      glm::vec4 clipCoords(ndcX, ndcY, -1.0f, 1.0f);
      glm::vec4 eyeCoords = glm::inverse(viewMatrix) * clipCoords;
      glm::vec3 rayDirection = -glm::normalize(glm::vec3(eyeCoords));

      glm::vec3 color = traceRay(globalCameraPosition, rayDirection, scene);

      image(x, y, 0, 0) = static_cast<unsigned char>(color.r * 255);
      image(x, y, 0, 1) = static_cast<unsigned char>(color.g * 255);
      image(x, y, 0, 2) = static_cast<unsigned char>(color.b * 255);
    }
  }
      
  display.render(image);
  display.paint();

  glm::vec3 originalPosition = scene[0].sphere.center;
  
  while (!display.is_closed()) {
    if (display.key()) {
      Shape& object = scene[0];

      switch (display.key()) {
        case cimg_library::cimg::keyARROWLEFT:
	  applyTranslation(scene, 0, glm::vec3(0.5f, 0.0f, 0.0f));
	  break;
	case cimg_library::cimg::keyARROWRIGHT:
	  applyTranslation(scene, 0, glm::vec3(-0.5f, 0.0f, 0.0f));
	  break;
	case cimg_library::cimg::keyARROWUP:
	  applyTranslation(scene, 0, glm::vec3(0.0f, 0.0f, -0.5f));
	  break;
	case cimg_library::cimg::keyARROWDOWN:
	  applyTranslation(scene, 0, glm::vec3(0.0f, 0.0f, 0.5f));
	  break;
        case cimg_library::cimg::keyQ:
	  applyRotation(scene, 0, 20.0, glm::vec3(0.0f, 1.0f, 0.0f));
	  break;
        case cimg_library::cimg::keyE:
	  applyRotation(scene, 0, -20.0, glm::vec3(0.0f, 1.0f, 0.0f));
	  break;
      }
      printMatrix(scene[0].getTransform());

      for (int y = 0; y < height; y++) {
	for (int x = 0; x < width; x++) {
	  float ndcX = aspectRatio * ((2.0f * x) / width - 1.0f);
	  float ndcY = 1.0f - (2.0f * y) / height;
	  glm::vec4 clipCoords(ndcX, ndcY, -1.0f, 1.0f);
	  glm::vec4 eyeCoords = glm::inverse(viewMatrix) * clipCoords;
	  glm::vec3 rayDirection = -glm::normalize(glm::vec3(eyeCoords));

	  glm::vec3 color = traceRay(globalCameraPosition, rayDirection, scene);

	  image(x, y, 0, 0) = static_cast<unsigned char>(color.r * 255);
	  image(x, y, 0, 1) = static_cast<unsigned char>(color.g * 255);
	  image(x, y, 0, 2) = static_cast<unsigned char>(color.b * 255);
	}
      }
      
      display.render(image);
      display.paint();
    }
    display.wait();
    }
  
  return 0;
}
