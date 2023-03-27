#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

int main(int argc, char** argv)
{
    // Load image
    cv::Mat img = cv::imread("../calibration/image1.jpg");

    // Create visualization window
    cv::viz::Viz3d window("Icosahedron");

    // Create icosahedron mesh
    cv::viz::Mesh icosahedron = cv::viz::Mesh::createIcosahedron();

    // Create widget for the icosahedron
    cv::viz::WMesh icosahedron_widget(icosahedron);
    icosahedron_widget.setColor(cv::viz::Color::green());

    // Set camera position and orientation
    cv::Affine3f cam_pose(cv::viz::makeCameraPose(cv::Vec3f(0.0f, 0.0f, 5.0f), cv::Vec3f(0.0f, 0.0f, 0.0f), cv::Vec3f(0.0f, 1.0f, 0.0f)));
    window.setViewerPose(cam_pose);

    // Add widgets to the window
    window.showWidget("Icosahedron", icosahedron_widget);

    // Start event loop
    while (!window.wasStopped()) {
        // Render icosahedron on image
        cv::Mat render_img;
        window.spinOnce(1, true);
        window.getScreenshot(render_img);
        cv::imshow("Icosahedron on Image", img + render_img);
        cv::waitKey(1);
    }

    return 0;
}
