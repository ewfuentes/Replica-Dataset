// Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
#include <PTexLib.h>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <atomic>
#include <sstream>
#include <thread>

#include <pangolin/display/display.h>
#include <pangolin/display/widgets/widgets.h>

#include "GLCheck.h"
#include "MirrorRenderer.h"

std::atomic<bool> should_save = false;

std::ostream &operator<<(std::ostream &out, const Eigen::Affine3d &mat) {
  out << mat.matrix();
  return out;
}

struct FloatingHandler: pangolin::Handler {
  FloatingHandler(pangolin::OpenGlRenderState &render_state, std::optional<std::filesystem::path> output_dir) : 
    render_state(&render_state), output_dir(output_dir){
    if (output_dir.has_value()) {
std::filesystem::create_directories(output_dir.value() / "color");
      std::filesystem::create_directories(output_dir.value() / "pose");
    }
  }

  void Keyboard(pangolin::View &, unsigned char key, int x, int y, bool pressed) {
    const int key_int = static_cast<int>(key);
    if (!pressed) {
      return;
    }

    constexpr double STEP = 0.2;
    const Eigen::Translation3d new_camera_from_old_camera([&]() -> Eigen::Vector3d{
      if (key == 'w') {
        return -Eigen::Vector3d::UnitZ() * STEP;
      } else if (key == 's') {
        return Eigen::Vector3d::UnitZ() * STEP;
      } else if (key == 'a') {
        return Eigen::Vector3d::UnitX() * STEP;
      } else if (key == 'd') {
        return -Eigen::Vector3d::UnitX() * STEP;
      } else if (key == 'q') {
        return Eigen::Vector3d::UnitY() * STEP;
      } else if (key == 'e') {
        return -Eigen::Vector3d::UnitY() * STEP;
      } else {
        return Eigen::Vector3d::Zero();
      }
    }());

    const Eigen::Affine3d new_camera_from_world = new_camera_from_old_camera * static_cast<Eigen::Affine3d>(render_state->GetModelViewMatrix()) ;

    render_state->SetModelViewMatrix(new_camera_from_world);

    if (output_dir.has_value()) {
      should_save.store(true);
    }
  }

  void Mouse(pangolin::View &view, const pangolin::MouseButton button, const int x, const int y, const bool pressed, int button_state) {
    std::cout << "mc pressed: " << pressed << " button state: " << button_state << std::endl;

    Eigen::Vector3d pt_in_cam = Eigen::Vector3d::Zero();
    view.GetCamCoordinates(*render_state, x, y, 0.1, pt_in_cam.x(), pt_in_cam.y(), pt_in_cam.z());
    if (!pressed && ((button_state & 0x01) == 0)) {
      last_mouse_on = std::nullopt;
      if (output_dir.has_value()) {
        should_save.store(true);
      }
    } else if (button == pangolin::MouseButton::MouseButtonLeft) {
      last_mouse_on = {
        .start_axis = pt_in_cam.normalized(),
        .camera_from_world = static_cast<Eigen::Affine3d>(render_state->GetModelViewMatrix()),
      };
    }
  }

  void MouseMotion(pangolin::View &view, const int x, const int y, int button_state) {
    if (last_mouse_on.has_value()) {
      Eigen::Vector3d pt_in_cam = Eigen::Vector3d::Zero();
      view.GetCamCoordinates(*render_state, x, y, 0.1, pt_in_cam.x(), pt_in_cam.y(), pt_in_cam.z());
      Eigen::Vector3d end_axis = pt_in_cam.normalized();

      const Eigen::Vector3d cross = last_mouse_on->start_axis.cross(end_axis);
      const double angle_rad = std::asin(cross.norm());
      const Eigen::Vector3d axis_in_camera = cross.normalized();
      if (angle_rad < 1e-3) {
        return;
      }

      const Eigen::AngleAxis<double> new_camera_from_old_camera(angle_rad, axis_in_camera);

      const Eigen::Affine3d new_camera_from_world = 
        new_camera_from_old_camera * last_mouse_on->camera_from_world;
      render_state->SetModelViewMatrix(new_camera_from_world);
    }

  }


  struct MouseOnState {
    Eigen::Vector3d start_axis;
    Eigen::Affine3d camera_from_world;
  };

  pangolin::OpenGlRenderState *render_state;
  std::optional<MouseOnState> last_mouse_on;
  std::optional<std::filesystem::path> output_dir;
};

int main(int argc, char* argv[]) {

  ASSERT(argc == 2 || argc == 3, "Usage: ./ReplicaViewer scene_dir [output_dir]");
  const std::filesystem::path scene_dir = argv[1];
  const std::filesystem::path mesh_file = scene_dir / "mesh.ply";
  const std::filesystem::path atlas_folder = scene_dir / "textures";
  const std::filesystem::path surface_file = scene_dir / "glass.sur";

  std::optional<std::filesystem::path> output_path = argc == 3 ? std::make_optional(argv[2]) : std::nullopt;


  ASSERT(std::filesystem::exists(mesh_file));
  ASSERT(std::filesystem::exists(atlas_folder));

  const int uiWidth = 180;
  const int width = 1280;
  const int height = 960;

  // Setup OpenGL Display (based on GLUT)
  pangolin::CreateWindowAndBind("ReplicaViewer", uiWidth + width, height);

  if (glewInit() != GLEW_OK) {
    pango_print_error("Unable to initialize GLEW.");
  }

  if(!checkGLVersion()) {
    return 1;
  }

  // Setup default OpenGL parameters
  glEnable(GL_DEPTH_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  const GLenum frontFace = GL_CW;
  glFrontFace(frontFace);
  glLineWidth(1.0f);

  // Tell the base view to arrange its children equally
  if (uiWidth != 0) {
    pangolin::CreatePanel("ui").SetBounds(0, 1.0f, 0, pangolin::Attach::Pix(uiWidth));
  }

  pangolin::View& container =
      pangolin::CreateDisplay().SetBounds(0, 1.0f, pangolin::Attach::Pix(uiWidth), 1.0f);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrixRDF_TopLeft(
          width,
          height,
          width / 2.0f,
          width / 2.0f,
          (width - 1.0f) / 2.0f,
          (height - 1.0f) / 2.0f,
          0.1f,
          100.0f),
      pangolin::ModelViewLookAtRDF(0, 0, 4, 0, 0, 0, 0, 1, 0));

  FloatingHandler s_handler(s_cam, output_path);

  pangolin::View& meshView = pangolin::Display("MeshView")
                                 .SetBounds(0, 1.0f, 0, 1.0f, (double)width / (double)height)
                                 .SetHandler(&s_handler);

  container.AddDisplay(meshView);

  // load mirrors
  std::vector<MirrorSurface> mirrors;
  if (std::filesystem::exists(surface_file)) {
    std::ifstream file(surface_file);
    picojson::value json;
    picojson::parse(json, file);

    for (size_t i = 0; i < json.size(); i++) {
      mirrors.emplace_back(json[i]);
    }
    std::cout << "Loaded " << mirrors.size() << " mirrors" << std::endl;
  }

  const std::string shadir = STR(SHADER_DIR);
  MirrorRenderer mirrorRenderer(mirrors, width, height, shadir);

  // load mesh and textures
  PTexMesh ptexMesh(mesh_file, atlas_folder);

  pangolin::Var<float> exposure("ui.Exposure", 0.01, 0.0f, 0.1f);
  pangolin::Var<float> gamma("ui.Gamma", ptexMesh.Gamma(), 1.0f, 3.0f);
  pangolin::Var<float> saturation("ui.Saturation", ptexMesh.Saturation(), 0.0f, 2.0f);
  pangolin::Var<float> depthScale("ui.Depth_scale", 0.1f, 0.0f, 1.0f);

  pangolin::Var<bool> wireframe("ui.Wireframe", false, true);
  pangolin::Var<bool> drawBackfaces("ui.Draw_backfaces", false, true);
  pangolin::Var<bool> drawMirrors("ui.Draw_mirrors", true, true);
  pangolin::Var<bool> drawDepth("ui.Draw_depth", false, true);

  ptexMesh.SetExposure(exposure);

  std::deque<std::jthread> background_tasks;

  while (!pangolin::ShouldQuit()) {
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    if (exposure.GuiChanged()) {
      ptexMesh.SetExposure(exposure);
    }

    if (gamma.GuiChanged()) {
      ptexMesh.SetGamma(gamma);
    }

    if (saturation.GuiChanged()) {
      ptexMesh.SetSaturation(saturation);
    }

    if (meshView.IsShown()) {
      meshView.Activate(s_cam);

      if (drawBackfaces) {
        glDisable(GL_CULL_FACE);
      } else {
        glEnable(GL_CULL_FACE);
      }

      if (wireframe) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0, 1.0);
        ptexMesh.Render(s_cam);
        glDisable(GL_POLYGON_OFFSET_FILL);
        // render wireframe on top
        ptexMesh.RenderWireframe(s_cam);
      } else if (drawDepth) {
        ptexMesh.RenderDepth(s_cam, depthScale);
      } else {
        ptexMesh.Render(s_cam);
      }

      glDisable(GL_CULL_FACE);

      if (drawMirrors) {
        for (size_t i = 0; i < mirrors.size(); i++) {
          MirrorSurface& mirror = mirrors[i];
          // capture reflections
          mirrorRenderer.CaptureReflection(mirror, ptexMesh, s_cam, frontFace, drawDepth, depthScale);

          // render mirror
          mirrorRenderer.Render(mirror, mirrorRenderer.GetMaskTexture(i), s_cam, drawDepth);
        }
      }

    }

    if (should_save.exchange(false) ) {
      std::cout << "Screenshot!" << std::endl;
      pangolin::RenderViews();

      std::stringstream ss;
      ss << std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

      pangolin::PixelFormat fmt = pangolin::PixelFormatFromString("RGBA32");
      pangolin::TypedImage buffer(meshView.v.w, meshView.v.h, fmt );
      glReadBuffer(GL_BACK);
      glPixelStorei(GL_PACK_ALIGNMENT, 1); // TODO: Avoid this?
      glReadPixels(meshView.v.l, meshView.v.b, meshView.v.w, meshView.v.h, GL_RGBA, GL_UNSIGNED_BYTE, buffer.ptr );


      background_tasks.emplace_front([buffer = std::move(buffer), fmt, file_name = ss.str(), 
                                     camera_from_world = s_cam.GetModelViewMatrix(), output_path](){
        pangolin::SaveImage(buffer, fmt, output_path.value() / "color" / (file_name + ".png"), false);

        std::ofstream out(output_path.value() / "pose" / (file_name + ".txt"));
        out << static_cast<Eigen::Matrix4d>(camera_from_world).inverse();
      });
      
      background_tasks.resize(100);
    }

    pangolin::FinishFrame();
  }

  return 0;
}
