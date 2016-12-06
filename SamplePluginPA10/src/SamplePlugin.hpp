#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../build/ui_SamplePlugin.h"
#include "helperFunctions.h"

#include <opencv2/opencv.hpp>
#include <rw/models/WorkCell.hpp>
#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

#include <fstream>
#include <sstream>

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	SamplePlugin();
	virtual ~SamplePlugin();

	virtual void open(rw::models::WorkCell* workcell);

    void restart();

	virtual void close();

	virtual void initialize();

    // Needs to be public in order to use in helper cpp file
    bool useVision = false;
    NoOfTargets No = multiple;
    static std::vector<rw::math::Vector2D<int>> cameraModel(std::vector<rw::math::Vector3D<double>> pointsInCameraFrame);
    static std::vector<rw::math::Vector3D<double>> getTargetsInCameraFrame
                                                    (rw::math::Transform3D<double> cameraTtexture,
                                                     std::vector<rw::math::Vector3D<double>> multipleTargOnTexture);

private slots:
	void btnPressed();
    void radioBtnToggled();
    void checkBoxStateChanged(int state);

	void timer();

	void stateChangedListener(const rw::kinematics::State& state);

private:
	static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

	QTimer* _timer;

	rw::models::WorkCell::Ptr _wc;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rw::math::Q from;
    std::string inputFileName;
    bool sim_dT_running;

    rw::math::Transform3D<double> worldTmarker_Default;


    rw::kinematics::Frame *textureFrame, *cameraFrame;
    rw::kinematics::MovableFrame *markerFrame;
    std::vector<rw::math::Vector2D<int>> targetPixelsReference;
    rw::math::Q velocity_limits;
    std::vector<rw::math::Vector3D<double>> textureTargetCoordinates;



    std::vector<rw::math::Vector2D<int>> cameraSimulation();
    rw::math::Transform3D<double> moveMarker(rw::kinematics::MovableFrame* markerFrame, std::string line);

    rw::math::Q algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                                         const rw::math::Transform3D<double> baseTtool_desired);

    rw::math::Q algorithm2(std::vector<rw::math::Vector2D<int>> targetRealPixels);

    void sim_dTs();
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
