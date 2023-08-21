// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5;       //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good()){
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }

    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;

    vertices.resize(nv, 3);
    facets.resize(nf, 3);

    for (int i = 0; i < nv; ++i){
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i){
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    //TODO: setup uniform

    //TODO: setup camera, compute w, u, v
    const Vector3d w = -1 * camera_gaze.normalized();
    const Vector3d u = camera_top.cross(w).normalized();
    const Vector3d v = w.cross(u);

    //TODO: compute the camera transformation
    Matrix4d slow;
    Matrix4d proj;
    
    slow << u(0), v(0), w(0), camera_position(0),
        u(1), v(1), w(1), camera_position(1),
        u(2), v(2), w(2), camera_position(2),
        0, 0, 0, 1;

    //TODO: setup projection matrix
    proj << 2 / (((near_plane * tan(field_of_view / 2.0)) * aspect_ratio) - (-((near_plane * tan(field_of_view / 2.0)) * aspect_ratio))), 0, 0, -(((near_plane * tan(field_of_view / 2.0)) * aspect_ratio) + (-((near_plane * tan(field_of_view / 2.0)) * aspect_ratio))) / (((near_plane * tan(field_of_view / 2.0)) * aspect_ratio) - (-((near_plane * tan(field_of_view / 2.0)) * aspect_ratio))),
        0, 2 / ((near_plane * tan(field_of_view / 2.0)) - (-(near_plane * tan(field_of_view / 2.0)))), 0, -((near_plane * tan(field_of_view / 2.0)) + (-(near_plane * tan(field_of_view / 2.0)))) / ((near_plane * tan(field_of_view / 2.0)) - (-(near_plane * tan(field_of_view / 2.0)))),
        0, 0, 2 / (-near_plane - (-far_plane)), -(-near_plane + (-far_plane)) / (-near_plane - (-far_plane)),
        0, 0, 0, 1;

    Matrix4d P;
    if (is_perspective){
        //TODO setup prespective camera
        P << (-near_plane), 0, 0, 0,
            0, (-near_plane), 0, 0,
            0, 0, ((-near_plane) + (-far_plane)), (-(-far_plane) * (-near_plane)),
            0, 0, 1, 0;

        uniform.view = proj * P.cast<double>() * (slow.inverse());
    }
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes slower;
        slower.position = uniform.view * va.position;
        return slower;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets

    //rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);

    for (int i = 0; i < facets.rows(); i++) {
        Eigen::Vector3d v1 = vertices.row(facets(i, 0)).transpose();
        Eigen::Vector3d v2 = vertices.row(facets(i, 1)).transpose();
        Eigen::Vector3d v3 = vertices.row(facets(i, 2)).transpose();

        vertex_attributes.push_back(VertexAttributes(v1(0), v1(1), v1(2)));
        vertex_attributes.push_back(VertexAttributes(v2(0), v2(1), v2(2)));
        vertex_attributes.push_back(VertexAttributes(v3(0), v3(1), v3(2)));
    }

    double aspect_ratio = static_cast<double>(frameBuffer.cols()) / static_cast<double>(frameBuffer.rows());
    Eigen:Matrix4d view = Eigen::Matrix4d::Identity();

    if (aspect_ratio < 1) {
        view(0, 0) = aspect_ratio;
    }else {
        view(1, 1) = 1 / aspect_ratio;
    }

    uniform.view = view;

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d rotation_matrix;
    rotation_matrix << (cos(alpha)), 0, (sin(alpha)), 0,
                        0, 1, 0, 0,
                        -(sin(alpha)), 0, (cos(alpha)), 0,
                        0, 0, 0, 1;

    Matrix4d identity_matrix;
    identity_matrix.setIdentity();

    return rotation_matrix * identity_matrix;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes slow;
        slow.position = uniform.view * va.position;
        return slow;
        return va;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0], fa.color[1], fa.color[2], fa.color[3]);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    //TODO: use the transformation matrix

    for (int i = 0; i < facets.rows(); ++i) {
        Vector3i vertices_index;
        vertices_index << facets.row(i).transpose();

        // add edges to vertex attributes
        for (int j = 0; j < 3; ++j) {
            Vector3d v1 = vertices.row(vertices_index(j)).transpose();
            Vector3d v2 = vertices.row(vertices_index((j + 1) % 3)).transpose();
            vertex_attributes.emplace_back(v1[0], v1[1], v1[2]);
            vertex_attributes.emplace_back(v2[0], v2[1], v2[2]);
        }
    }

    uniform.view = uniform.view * trafo.cast<double>();

    frameBuffer.setZero();

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal
        //TODO: compute the correct lighting
        VertexAttributes slow;
        slow.position = uniform.view * va.position;
        slow.normal = va.normal;
        Vector3d vcoller(0, 0, 0);

        for (int i = 0; i < light_positions.size(); i++) {
            const Vector3d light_position = light_positions[i];
            const Vector3d light_color = light_colors[i];

            Vector3d p(slow.position[0], slow.position[1], slow.position[2]);
            Vector3d N(va.normal[0], va.normal[1], va.normal[2]);

            // diffuse contribution
            const Vector3d diffuse = obj_diffuse_color * std::max(((light_position - p).normalized()).dot(N), 0.0);

            // specular contribution
            const Vector3d specular = obj_specular_color * std::pow(std::max(N.dot((((light_position - p).normalized()) - p).normalized()), 0.0), obj_specular_exponent);

            // attenuate lights based on distance^2 to light source
            vcoller += (diffuse + specular).cwiseProduct(light_color) / (light_position - p).squaredNorm();
        }
        vcoller += ambient_light;

        Vector4d C(vcoller[0], vcoller[1], vcoller[2], 1);
        slow.color = C;

        return slow;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        FragmentAttributes slow(va.color[0], va.color[1], va.color[2], uniform.color[3]);
        Vector4d loki(va.position[0], va.position[1], -va.position[2], va.position[3]);

        if (is_perspective) {
            loki[2] = va.position[2];
        }
        slow.position = loki;
        return slow;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        FrameBufferAttributes out = previous;
        if (fa.position(2) < previous.depth) {
            out = FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
        }

        return out;
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: compute the normals
    //TODO: set material colors

    for (int i = 0; i < facets.rows(); i++) {
        // Extract the three vertices of the current facet
        Vector3d vertex_a = vertices.row(facets(i, 0)).transpose();
        Vector3d vertex_b = vertices.row(facets(i, 1)).transpose();
        Vector3d vertex_c = vertices.row(facets(i, 2)).transpose();

        // Create three vertex attributes with the same normal
        VertexAttributes a(vertex_a[0], vertex_a[1], vertex_a[2]);
        a.normal = (vertex_b - vertex_a).cross(vertex_c - vertex_a).normalized().cast<double>();
        VertexAttributes b(vertex_b[0], vertex_b[1], vertex_b[2]);
        b.normal = (vertex_b - vertex_a).cross(vertex_c - vertex_a).normalized().cast<double>();
        VertexAttributes c(vertex_c[0], vertex_c[1], vertex_c[2]);
        c.normal = (vertex_b - vertex_a).cross(vertex_c - vertex_a).normalized().cast<double>();

        // Add the three vertex attributes to the list
        vertex_attributes.emplace_back(a);
        vertex_attributes.emplace_back(b);
        vertex_attributes.emplace_back(c);
    }

    double aspect_ratio = double(frameBuffer.cols()) / double(frameBuffer.rows());
    uniform.view *= trafo.cast<double>();

    uniform.color = {0, 0, 0, 1};

    frameBuffer.setZero();
    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);

    uniform.view *= trafo.cast<double>();

    //TODO: compute the vertex normals as vertex normal average

    std::vector<VertexAttributes> vertex_attributes;
    
    //TODO: create vertex attributes
    //TODO: set material colors
    std::vector<Vector3d> Normals(vertices.rows(), Vector3d(0, 0, 0));

    for (int i = 0; i < facets.rows(); ++i)
    {
        Vector3d v1 = vertices.row(facets(i, 0)).transpose();
        Vector3d v2 = vertices.row(facets(i, 1)).transpose();
        Vector3d v3 = vertices.row(facets(i, 2)).transpose();

        Normals[facets(i, 0)] += (v3 - v1).cross(v2 - v1).normalized().cast<double>();
        Normals[facets(i, 1)] += (v3 - v1).cross(v2 - v1).normalized().cast<double>();
        Normals[facets(i, 2)] += (v3 - v1).cross(v2 - v1).normalized().cast<double>();

        VertexAttributes attr1(v1[0], v1[1], v1[2]);
        VertexAttributes attr2(v2[0], v2[1], v2[2]);
        VertexAttributes attr3(v3[0], v3[1], v3[2]);

        attr1.normal = Normals[facets(i, 0)].normalized();
        attr2.normal = Normals[facets(i, 1)].normalized();
        attr3.normal = Normals[facets(i, 2)].normalized();

        vertex_attributes.push_back(attr1);
        vertex_attributes.push_back(attr2);
        vertex_attributes.push_back(attr3);
    }

    uniform.color = {0, 0, 0, 1};
    frameBuffer.setZero();

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[]){
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    //TODO: add the animation
    const std::vector<std::string> gifNames = {"wireframe_render.gif", "flat_shading.gif", "pv_shading.gif"};
    const int delay = 25;
    using RenderFunc = std::function<void(double, FrameBuffer&)>;
    const std::vector<RenderFunc> renderFuncs = {wireframe_render, flat_shading, pv_shading};
    
    // iterate each gif filename
    for (size_t i = 0; i < gifNames.size(); ++i) {
        const std::string& gifName = gifNames[i];
        const RenderFunc& renderFunc = renderFuncs[i];

        // create the gif file
        const int delay = 25;
        GifWriter g;
        int rows = frameBuffer.rows();
        int cols = frameBuffer.cols();
        GifBegin(&g, gifName.c_str(), rows, cols, delay);

        for (int i = 0; i < 13; ++i) {
            double angle = 1 + (EIGEN_PI / 12) * i;
            frameBuffer.setConstant(FrameBufferAttributes());
            renderFunc(angle, frameBuffer);
            framebuffer_to_uint8(frameBuffer, image);
            GifWriteFrame(&g, image.data(), rows, cols, delay);
        }

        GifEnd(&g);
    }

    return 0;
}
