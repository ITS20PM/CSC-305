// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

// helper function

// get points on the sphere 
/*  return a point in 3d space where a ray intersects with a sphere as long 
as there exists one valid t that is positive. */
Vector3d get_Point_Sphere(Vector3d c, Vector3d e, Vector3d d, double R) {
    // coefficient of quadratic function Ax^2 + Bx + C
    double A = d.dot(d);
    double B = (2 * d).dot(e - c);
    double C = (e - c).dot(e - c) - (R * R);

    double discriminant = (B * B) - (4 * A * C);

    double t;

    if (discriminant == 0)  {
        // 1 intersection point
        t = (-B + sqrt(discriminant)) / (2 * A);
    }else if (discriminant > 0) {
        // 2 intersection point
        t = (-B + sqrt(discriminant)) / (2 * A);
        double t2 = (-B - sqrt(discriminant)) / (2 * A);
        
        // Because in the check we know both or one of the t's are positive.
        if (t < 0){
            t = t2;
            // if t is the negative one we know t2 is the positive
        }else if (t > 0 && t2 > 0){
            // if both t's are valid for intersection then we gotta find the smaller one
            t = fmin(t, t2);
        }
    }
    // At this point we know the correct t for the intersect.

    return (e + (t * d));
}

// rays on the sphere
bool ray_sphere(Vector3d c, Vector3d e, Vector3d d, double R) {
    double A = d.dot(d);
    double B = (2 * d).dot(e - c);
    double C = (e - c).dot(e - c) - (R * R);

    double discriminant = (B * B) - (4 * A * C);

    if (discriminant < 0){
        return false;
    }else if (discriminant == 0) {
        // 1 intersection point
        double t = (-B + sqrt(discriminant)) / (2 * A);

        if (t < 0){
            return false;
        }
    }else if (discriminant > 0){
        // More than one intersection point, in this case two
        double t = (-B + sqrt(discriminant)) / (2 * A);
        double t2 = (-B - sqrt(discriminant)) / (2 * A);

        if (t < 0 && t2 < 0){
            return false;
        }
    }
    return true;
}

// paralleloogram
Vector3d getPoint(Vector3d u_vector, Vector3d v_vector, Vector3d d_vector, Vector3d a_vector, Vector3d e_vector) {
    // Initialize and declare matrix A
    Matrix3d A;
    A << -u_vector, -v_vector, d_vector;

    // vector from a - e
    Vector3d ae_vector = a_vector - e_vector;

    // Calculate the values for solution
    Vector3d solution_vector = A.colPivHouseholderQr().solve(ae_vector);

    return a_vector + (solution_vector(0) * u_vector) + (solution_vector(1) * v_vector);
}

// ray tracing intersects
bool ray_intersect(Vector3d u_vector, Vector3d v_vector, Vector3d d_vector, Vector3d a_vector, Vector3d e_vector) {
    // Initialize and declare matrix A
    Matrix3d A;
    A << -u_vector, -v_vector, d_vector;
    
    // Create vector = a - e
    Vector3d ae_vector = a_vector - e_vector;

    // Calculate the values for solution
    Vector3d solution_vector = A.colPivHouseholderQr().solve(ae_vector);

    // Check t
    if (solution_vector(2) < 0){
        return false;
    }

    // Check u
    if (solution_vector(0) < 0 || solution_vector(0) > 1){
        return false;
    }

    // Check 0 <= v <= 1
    if (solution_vector(1) < 0 || solution_vector(1) > 1){
        return false;
    }

    // If all conditions above are false, return true
    return true;
}

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius){
                // when the ray hits the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0); 
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i){
        for (unsigned j = 0; j < C.rows(); ++j){
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // TODO: Check if the ray intersects with the parallelogram
            if (ray_intersect(pgram_u, pgram_v, ray_direction, pgram_origin, ray_origin)){
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = getPoint(pgram_u, pgram_v, ray_direction, pgram_origin, ray_origin);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (pgram_v.cross(pgram_u)).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i){
        for (unsigned j = 0; j < C.rows(); ++j){
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = pixel_center - camera_origin;

            // TODO: Check if the ray intersects with the parallelogram
            if (ray_intersect(pgram_u, pgram_v, ray_direction, pgram_origin, ray_origin)){
                // TODO: The ray hit the parallelogram, compute the exact intersection point
                // pt
                Vector3d ray_intersection = getPoint(pgram_u, pgram_v, ray_direction, pgram_origin, ray_origin);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = (pgram_v.cross(pgram_u)).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd R = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd G = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd B = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0, 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < C.cols(); ++i){
        for (unsigned j = 0; j < C.rows(); ++j){
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = pixel_center - camera_origin;

            // Intersect with the sphere
            // TODO: implement the generic ray sphere intersection
            if (ray_sphere(sphere_center, ray_origin, ray_direction, sphere_radius)){
                // TODO: The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection = get_Point_Sphere(sphere_center, ray_origin, ray_direction, sphere_radius);

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = ((sphere_center - ray_intersection) / sphere_radius).normalized();

                // TODO: Add shading parameter here
                Vector3d res1 = (ray_intersection - light_position).normalized();
                Vector3d res2 = (ray_intersection - ray_origin).normalized();
                Vector3d res3 = ((res2 + res1) / ((res2 + res1).norm())).normalized();

                Vector3d diffuse = diffuse_color * 1 * fmax(0, (ray_normal.dot(res1)));
                Vector3d specular = specular_color * 1 * pow(fmax(0, (ray_normal.dot(res3))), specular_exponent);

                // Simple diffuse model
                R(i, j) = ambient + diffuse(0) + specular(0);
                G(i, j) = ambient + diffuse(1) + specular(1);
                B(i, j) = ambient + diffuse(2) + specular(2);

                // Clamp to zero
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, R, B, A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
