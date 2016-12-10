#include <iostream>
#include <vector>
#include <string>
#include <rw/rw.hpp>
#include <rw/math/Transform3D.hpp>

#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <QPushButton>
#include <QRadioButton>
#include <QSpinBox>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <QTimer>

#include <boost/bind.hpp>
#include <QtPlugin>
#include <QFileDialog>
#include <QString>
#include <chrono>

#include "inverseKinematics.hpp"
#include "marker1.h"

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rws;

using namespace cv;

std::string file;
std::ifstream infile;
std::ofstream writeJoints;
std::ofstream writeJointsVelocities;
std::ofstream writeToolPose;
std::ofstream writeImCorErrors;
std::ofstream write_dU;
std::string outFileMark;


std::stringstream ss;

const string deviceName = "PA10";
Device::Ptr device;

int max_dT = 20, min_dT = 5;
double computationTimeTotal = 0;
double imgRecognitionTimeTotal = 0;
int sequenceSteps = 0;
double deltaT =  max_dT; // timer period in ms
double t = 0;
double maxEucD = 0;
boost::numeric::ublas::vector<double> max_dU_Image (2);

//Frame* base;
//Frame* world;


SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/SamplePluginPA10/src/pa_icon.png"))
{
	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
        
         // Set a new texture (one pixel = 1 mm)
        Image::Ptr image;
        image = ImageLoader::Factory::load("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/SamplePluginPA10/markers/Marker1.ppm");
        _textureRender->setImage(*image);
        image = ImageLoader::Factory::load("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/SamplePluginPA10/backgrounds/color3.ppm");
        _bgRender->setImage(*image);
	_framegrabber = NULL;
    
        setupUi(this);

	_timer = new QTimer(this);
        connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn_Restart   ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_Select, SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn_Start    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_dT_Sim, SIGNAL(pressed()), this, SLOT(btnPressed()));
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );
    connect(_track1Pt ,SIGNAL(toggled(bool)), this, SLOT(radioBtnToggled()) );
    connect(_trackMPt ,SIGNAL(toggled(bool)), this, SLOT(radioBtnToggled()) );
    connect(_imgRec, SIGNAL(stateChanged(int)), this, SLOT(checkBoxStateChanged(int)));
    //connect(_slider,SIGNAL(valueChanged()), this, SLOT(sliderValueChanged()) );
        
}

SamplePlugin::~SamplePlugin()
{
    /* deallocate used memory */
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
    /* do something when plugin is initialized */
    
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/PA10WorkCell/ScenePA10RoVi1.wc.xml");
	getRobWorkStudio()->setWorkCell(wc);

	// Load Lena image
	Mat im, image;
	im = imread("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/SamplePluginPA10/src/lena.bmp", CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
    QImage img(im.data, im.cols, im.rows, im.step, QImage::Format_RGB888);
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
    /* do something when workcell is openned */
    
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();

	log().info() << workcell->getFilename() << "\n";

	if (_wc != NULL) {
            // Add the texture render to this workcell if there is a frame for texture
            textureFrame = _wc->findFrame("MarkerTexture");
            if (textureFrame != NULL) {

                    getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
            }
            // Add the background render to this workcell if there is a frame for texture
            Frame* bgFrame = _wc->findFrame("Background");
            if (bgFrame != NULL) {
                    getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
            }
            
            // Find manipulator Device
            device = _wc->findDevice(deviceName);
            if (device == NULL) {
                log().info() << "Device: " << deviceName << " not found!\n";
            }
            // Get Manipulator to the initial state
            from = rw::math::Q(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
            device->setQ(from, _state);

            // Create a GLFrameGrabber if there is a camera frame with a Camera property set
            cameraFrame = _wc->findFrame("CameraSim");
            if (cameraFrame != NULL) {
                if (cameraFrame->getPropertyMap().has("Camera")) {
                    // Read the dimensions and field of view
                    double fovy;
                    int width,height;
                    std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                    std::istringstream iss (camParam, std::istringstream::in);
                    iss >> fovy >> width >> height;
                    // Create a frame grabber
                    _framegrabber = new GLFrameGrabber(width,height,fovy);
                    SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                    _framegrabber->init(gldrawer);

                    _framegrabber->grab(cameraFrame, _state);       //grab image from simulated camera
                    const Image &image = _framegrabber->getImage(); //save image for first image processing

                    // Convert to OpenCV image
                    Mat im = toOpenCVImage(image);
                    Mat imFlipLabel;

                    //After opening plugin -> initialize to follow multiple points
                    outFileMark = "M_Targ_Pts";
                    No = multiple;
                    useVision = false;

                    if (useVision)
                    {
                        Mat imflip;
                        cv::flip(im, imflip, -1);

                        // Get target points from image
                        targetPixelsReference = marker1(imflip, No);

                        // Show Image with targetPixels in Qlabel
                        // Flip image to fit into Qlabel
                        cv::flip(imflip, imFlipLabel, 1);
                    } else
                    {
                        // Get vector of target points coordinates in texture frame
                        textureTargetCoordinates = hardcodedTextureTargetCoordinates(No, textureFrame, cameraFrame, _state);

                        // Compute Transformation from cameraFrame to textureFrame
                        rw::math::Transform3D<double> cameraTtexture = rw::kinematics::Kinematics::frameTframe(cameraFrame, textureFrame, _state);

                        // Get vector of target points coordinates in camera frame
                        std::vector<rw::math::Vector3D<double>> targetsInCameraFrame = SamplePlugin::getTargetsInCameraFrame(cameraTtexture, textureTargetCoordinates);

                        // class variable storing pixel target position
                        targetPixelsReference = SamplePlugin::cameraModel(targetsInCameraFrame);

                        // Flip image to fit into Qlabel
                        cv::flip(im, imFlipLabel, 0);
                    }

                    // Show in QLabel
                    QImage img(imFlipLabel.data, imFlipLabel.cols, imFlipLabel.rows, imFlipLabel.step, QImage::Format_RGB888);
                    QPixmap p = QPixmap::fromImage(img);
                    unsigned int maxW = 400;
                    unsigned int maxH = 800;
                    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

                }
            }
            // Find and save reference to the Marker Frame
            markerFrame = (MovableFrame*) _wc->findFrame("Marker");
            // Save initial Transformation of markerFrame
            worldTmarker_Default = markerFrame->getTransform(_state); // save initial transformation to be able to restart plugin

            log().info() << "Number of Target Pixels: " << targetPixelsReference.size() << std::endl;
            log().info() << "Target pixel: " << targetPixelsReference[0] << ", ";
            //log().info() << targetPixelsReference[1] << ", ";
            //log().info() << targetPixelsReference[2];
            log().info() << "\n";

            // Save velocity limits
            velocity_limits = device->getVelocityLimits();
            joint_limits = device->getBounds();
            log().info() << "Velocity Limits: " << velocity_limits << "\n";
            log().info() << "Joint Limits (-): " << joint_limits.first << "\n";
            log().info() << "Joint Limits (+): " << joint_limits.second << "\n";

            // Default Sequence File
            inputFileName = "/media/petr/WD_HDD/SDU/RoVi1/FinalProject/SamplePluginPA10/motions/MarkerMotionFast.txt";

            // open stream for reading from file
            infile = std::ifstream(inputFileName);
	}
}

void SamplePlugin::restart()
{
    // Stop the timer if it already isn't
    if (_timer->isActive())
    {
        _timer->stop();
        log().info() << "Timer stopped\n";
    }
    _btn_Start->setText("Start Visual Sevoing");

    // Return file stream to the beggining of the file
    infile.clear();
    infile.seekg(0, std::ios::beg);

    // Put marker back to its initial possition
    markerFrame->setTransform(worldTmarker_Default,_state);
    // Set manipulator to inital state
    device->setQ(from, _state);

    // update RWStudio visualization
    getRobWorkStudio()->setState(_state);

    // Reload QImage and recompute target pixels
    _framegrabber->grab(cameraFrame, _state);       //grab image from simulated camera
    const Image& image = _framegrabber->getImage(); //save image for first image processing

    // Convert to OpenCV image
    Mat im = toOpenCVImage(image);
    Mat imFlipLabel;

    if (useVision)
    {
        Mat imflip;
        cv::flip(im, imflip, -1);

        // Get target points from image
        targetPixelsReference = marker1(imflip, No);

        // Show Image with targetPixels in Qlabel
        // Flip image to fit into Qlabel
        cv::flip(imflip, imFlipLabel, 1);
    } else
    {
        // Get vector of target points coordinates in texture frame
        textureTargetCoordinates = hardcodedTextureTargetCoordinates(No, textureFrame, cameraFrame, _state);

        // Compute Transformation from cameraFrame to textureFrame
        rw::math::Transform3D<double> cameraTtexture = rw::kinematics::Kinematics::frameTframe(cameraFrame, textureFrame, _state);

        // Get vector of target points coordinates in camera frame
        std::vector<rw::math::Vector3D<double>> targetsInCameraFrame = SamplePlugin::getTargetsInCameraFrame(cameraTtexture, textureTargetCoordinates);

        // class variable storing pixel target position
        targetPixelsReference = SamplePlugin::cameraModel(targetsInCameraFrame);

        // Flip image to fit into Qlabel
        cv::flip(im, imFlipLabel, 0);
    }

    // Show in QLabel
    QImage img(imFlipLabel.data, imFlipLabel.cols, imFlipLabel.rows, imFlipLabel.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    log().info() << "Number of Target Pixels: " << targetPixelsReference.size() << std::endl;


    // Clear maximum errors variables
    maxEucD = 0;
    max_dU_Image = boost::numeric::ublas::vector<double> (2);

    writeJoints.close();
    writeJointsVelocities.close();
    write_dU.close();
    writeToolPose.close();
    writeImCorErrors.close();
}

void SamplePlugin::close() {
    /* do something when the workcell is closed */
    
	log().info() << "CLOSE" << "\n";

	// Stop the timer
	_timer->stop();
	// Remove the texture render
	//Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;

    writeJoints.close();
    writeJointsVelocities.close();
    write_dU.close();
    writeToolPose.close();
    writeImCorErrors.close();
}


Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed()
{   // clickEvent
    QObject *obj = sender();
    if (obj == _btn_Restart)
    {
        log().info() << "Restart Plugin\n";
        SamplePlugin::restart();
    } else if (obj == _btn_Start)
    {
        log().info() << "Start Visual Sevoing\n";
        SamplePlugin::restart();
        // Toggle the timer on and off
        if (!_timer->isActive())
        {
            writeJoints.open("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/results/joints_" + outFileMark + ".csv");
            writeJointsVelocities.open("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/results/joints_velocities_" + outFileMark + ".csv");
            writeJoints << "time[s], Q0, Q1, Q2, Q3, Q4, Q5, Q6, Time for Inverse kinematics[s], Time for move[ms]"
                        << std::endl;
            writeJointsVelocities << "time[s], dQ0/dT, dQ1/dT, dQ2/dT, dQ3/dT, dQ4/dT, dQ5/dT, dQ6/dT" << std::endl;

            write_dU.open("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/results/dUImage_" + outFileMark + ".csv");
            write_dU << "time[s], Euc. Dist, dU1, dV1, dU2, dV2, dU3, dV3" << std::endl;

            writeToolPose.open("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/results/tool_pose_" + outFileMark + ".csv");
            writeToolPose << "time[s], x, y, z, R, P, Y" << std::endl;


            t = 0;
            // Write into the file
            writeJoints << t << ", " << from[0] << ", " << from[1] << ", " << from[2] << ", " << from[3] << ", "
                        << from[4] << ", " << from[5] << ", " << from[6] << ", 0, 0" << std::endl;
            writeJointsVelocities << t << ", 0, 0, 0, 0, 0, 0, 0" << std::endl;

            write_dU << "0, 0, 0, 0, 0, 0, 0, 0" << std::endl;
            auto worldTcamera = cameraFrame->wTf(_state);
            auto camPosition = worldTcamera.P();
            auto camOrientation = rw::math::RPY<double>(worldTcamera.R());
            writeToolPose << t << ", " << camPosition[0] << ", " << camPosition[1] << ", " << camPosition[2] << ", "
                          << camOrientation[0] << ", " << camOrientation[1] << ", " << camOrientation[2] << ", "
                          << std::endl;
            t = t + static_cast<double>(deltaT) / 1000;

            sim_dT_running = false;
            _timer->start(deltaT); // 100 -> run 10 Hz; or deltaT ???
            log().info() << "Timer started\n";
            _btn_Start->setText("Pause");

        } else
        {
            _timer->stop();
            _btn_Start->setText("Continue Visual Sevoing");
            log().info() << "Timer stopped\n";
        }
    } else if (obj == _btn_Select)
    {
        QString fileName = QFileDialog::getOpenFileName(this, tr("Open Sequence File"), "/home", "Text Files (*.txt)");
        inputFileName = fileName.toStdString();
    } else if (obj == _spinBox)
    {
        deltaT = _spinBox->value();
        log().info() << "deltaT:" << _spinBox->value() << "\n";
    } else if (obj == _btn_dT_Sim)
    {
        if (!_timer->isActive())
        {
            SamplePlugin::restart();
            std::size_t posStart = inputFileName.find("MarkerMotion");
            std::size_t posEnd = inputFileName.find(".txt");
            std::string file_prefix = inputFileName.substr(posStart, posEnd - posStart);
            log().info() << file_prefix << std::endl;
            writeImCorErrors.open("/media/petr/WD_HDD/SDU/RoVi1/FinalProject/results/" + file_prefix + "_img_error_" + outFileMark + ".csv");
            if (No == single)
            {
                writeImCorErrors << "deltaT[s], maxEuc_dU[pixels], max_dU[pixels], max_dV[pixels], ";
            } else
            {
                writeImCorErrors << "deltaT[s], maxEuc_dU_[pixels], max_dU_1[pixels], max_dV_1[pixels], ";
                writeImCorErrors << "max_dU_2[pixels], max_dV_2[pixels], max_dU_3[pixels], max_dV_3[pixels], ";
            }
            writeImCorErrors << "Avg Inv Kinem Time [ms], Avg Img Rec Time [ms]" << std::endl;
            deltaT = max_dT;
            sim_dT_running = true;
            SamplePlugin::sim_dTs();
        }
    } else if (obj == _imgRec)
    {
        SamplePlugin::restart();
    }
}

void SamplePlugin::sim_dTs()
{
    // Return file stream to the beggining of the file
    infile.clear();
    infile.seekg(0, std::ios::beg);

    computationTimeTotal = 0;
    imgRecognitionTimeTotal = 0;
    sequenceSteps = 0;

    if (deltaT >= min_dT)
    {
        // Put marker back to its initial possition
        markerFrame->setTransform(worldTmarker_Default,_state);
        // Set manipulator to inital state
        device->setQ(from, _state);

        maxEucD = 0;
        max_dU_Image = boost::numeric::ublas::vector<double> (2);

        _timer->start(deltaT); //100 or deltaT

    } else{
        writeImCorErrors.close();
    }

}
void SamplePlugin::radioBtnToggled()
{
    log().info() << "Radio Btn toogled\n";
    if (_trackMPt->isChecked())
    {
        //currentTargOnTexture = &multipleTargOnTexture;
        log().info() << "Mult Targets selected\n";
        outFileMark = "M_Targ_Pts";
        No = multiple;
    }
    else if(_track1Pt->isChecked())
    {
        //currentTargOnTexture = &singleTargOnTexture;
        log().info() << "Single Target selected\n";
        outFileMark = "1_Targ_Pt";
        No = single;
    }

    SamplePlugin::restart();
}

void SamplePlugin::checkBoxStateChanged(int state)
{
    // clickEvent
    QObject *obj = sender();
    if (obj == _imgRec)
    {
        useVision = static_cast<bool>(state);
        log().info() << "Image Recognition set to: " << useVision << std::endl;
    }
}


std::vector<rw::math::Vector3D<double>> SamplePlugin::getTargetsInCameraFrame(rw::math::Transform3D<double> cameraTtexture, std::vector<rw::math::Vector3D<double>> multipleTargOnTexture)
{
    std::vector<rw::math::Vector3D<double>> targetsInCameraFrame;
    int noPts = multipleTargOnTexture.size();
    for (int i = 0; i < noPts; ++i)
        targetsInCameraFrame.push_back(cameraTtexture * multipleTargOnTexture[i]);

    return targetsInCameraFrame;
}

std::vector<rw::math::Vector2D<int>> SamplePlugin::cameraModel(std::vector<rw::math::Vector3D<double>> pointsInCameraFrame)
{
    int f = 823; //focal length of camera in pixels
    int size = pointsInCameraFrame.size();

    rw::math::Vector3D<double> onePt;
    rw::math::Vector2D<int> pixelCoordinates;

    std::vector<rw::math::Vector2D<int>> pixCoordinatesVector;

    for (int i = 0; i < size; ++i)
    {
        onePt = pointsInCameraFrame[i];
        // Use camera model to calculate pixel coordinates of target point
        pixelCoordinates[0] = (f*onePt[0]) / onePt[2];
        pixelCoordinates[1] = (f*onePt[1]) / onePt[2];
        pixCoordinatesVector.push_back(pixelCoordinates);
    }

    return pixCoordinatesVector;
}




std::vector<rw::math::Vector2D<int>> SamplePlugin::cameraSimulation()
{
    // Simulate the image pixel location with transformations
    rw::math::Transform3D<double> cameraTtexture = rw::kinematics::Kinematics::frameTframe(cameraFrame, textureFrame, _state);

    // Get vector of target points coordinates in camera frame
    std::vector<rw::math::Vector3D<double>> targetsInCameraFrame = getTargetsInCameraFrame(cameraTtexture, textureTargetCoordinates);

    //log().info() << "Target In cam Frame: " << targetsInCameraFrame[0] << ",  ";
    //log().info() << targetsInCameraFrame[1] << ", ";
    //log().info() << targetsInCameraFrame[2];
    //log().info() << "\n";
    // class variable storing pixel target position
    std::vector<rw::math::Vector2D<int>> targetRealPixels = SamplePlugin::cameraModel(targetsInCameraFrame);
    //log().info() << "Target Real Pixels: " << targetRealPixels[0] << ", ";
    //log().info() << targetRealPixels[1] << ", ";
    //log().info() << targetRealPixels[2];
    //log().info() << "\n";

    return targetRealPixels;
}

rw::math::Q SamplePlugin::algorithm1(const rw::models::Device::Ptr device, rw::kinematics::State state, const rw::kinematics::Frame* tool,
                       const rw::math::Transform3D<double> baseTtool_desired) {
    auto baseTtool = device->baseTframe(tool, state);
    auto deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
    rw::math::Q q = device->getQ(state);
    log().info() << "state of manip. joints: " << q << "\n";
    const double epsilon = 0.01;
    while(deltaU.norm2() > epsilon) {
        auto J = device->baseJframe(tool, state);
        /*
         * Example, that J.e().inverse() doesn't work
        log().info() << "Jacobian: " << J << "\n";
        log().info() << "Jacobian Inverse: " << J.e().inverse() << "\n";
        log().info() << "deltaU: " << deltaU.e() << "\n";
        */
        // Calculate PseudoInverse of Jacobian; J.m() returns boost matrix type -> on this type it is possible to apply PSeudoinverse
        auto Jinv = rw::math::LinearAlgebra::pseudoInverse(J.m());
        boost::numeric::ublas::vector<double> boost_dU = deltaU.m();
        // Create new boost vector for change in joints dQ
        boost::numeric::ublas::vector<double> boost_dQ = boost::numeric::ublas::prod(Jinv, boost_dU );
        // Construct Q state from RW lib
        rw::math::Q deltaQ(boost_dQ);
        /*
         * Check that boost inverse works OK
        log().info() << "Jacobian Inverse2: " << Jinv << "\n";
        log().info() << "deltaU: " << deltaU.m() << "\n";
        log().info() << "Velocity: " << deltaQ << "\n";
        */
        q += deltaQ;
        device->setQ(q, state);
        baseTtool = device->baseTframe(tool, state);
        deltaU = calculateDeltaU(baseTtool, baseTtool_desired);
        log().info() << "deltaU norm = " << deltaU.norm2() << "\ncomputing new Q configuration\n";
    }

    return q;
}


rw::math::Q SamplePlugin::algorithm2(std::vector<rw::math::Vector2D<int>> targetRealPixels)
{

    //log().info() << "Size of targetRealPixels: " << targetRealPixels.size() << ", Size of targetPixelsReference: " << targetPixelsReference.size() << "\n";

    boost::numeric::ublas::vector<double> dU_Image = calculate_dUImage(targetRealPixels, targetPixelsReference);
    double eucD = euclideanDist(dU_Image);
    if (sim_dT_running)
    {
        if (eucD > maxEucD)
        {
            maxEucD = eucD;
            // store maximum dU_image errors in max variable
            max_dU_Image = boost::numeric::ublas::vector<double>(dU_Image);

            log().info() << "maxEucDist: " << eucD << ", with points: " << dU_Image << std::endl;
            log().info() << "maxDuSaved: " << max_dU_Image << std::endl;
        }
    }
    // Save errors into file
    write_dU << eucD;
    for(size_t i = 0; i < dU_Image.size(); i += 2)
        write_dU << ", " << dU_Image[0+i] << ", " << dU_Image[1+i];
    write_dU << std::endl;

    // Calculate image Jacobian
    auto J_image = calculateImageJ(targetRealPixels, 823);
    //log().info() << J_image << "\n";

    boost::numeric::ublas::matrix<double> Z_image = calculate_Z_image(device, _state, cameraFrame, J_image);

    boost::numeric::ublas::vector<double> boost_dQ = compute_dQ_LSM(Z_image, dU_Image);
    // Construct Q state from RW lib
    rw::math::Q dQ(boost_dQ);




    //targetRealPixels = SamplePlugin::cameraSimulation();
    //dU_Image = calculate_dUImage(targetRealPixels, targetPixelsReference);
    //log().info() << "deltaU norm = " << dU_Image.norm2() << "\ncomputing new Q configuration\n";

    return dQ;
}

rw::math::Transform3D<double> SamplePlugin::moveMarker(rw::kinematics::MovableFrame* markerFrame, std::string line)
{
    ss = std::stringstream(line);
    rw::math::Vector3D<double> position;
    rw::math::RPY<double> rotation;
    // get position
    for( int i = 0; i < 3; i++)
        ss >> position[i];
    // get rotation EEA angles
    for( int i = 0; i < 3; i++)
        ss >> rotation[i];
    //log().info() << "New Pos and Rot of the Marker: " << position << ", " << rotation << "\n";

    // create transformation matrix
    const rw::math::Transform3D<double> worldTmarker(position, rotation);
    //log().info() << "New T for World to Marker: \n" << worldTmarker << "\n";

    // move the Marker in Simulator World
    markerFrame->setTransform(worldTmarker, _state);

    // update RWStudio visualization
    getRobWorkStudio()->setState(_state);

    return worldTmarker;
}

void SamplePlugin::timer()
{
    // Move the marker in the workcell
    //-----------------------------------------------------------------------------
    if(markerFrame == nullptr) {
        RW_THROW("marker frame not found!");
        log().info() << "Marker not found\n";
    }
    // load transformation from the file
    std::string line;
    // If stream didnt reach end of the file - getline is returns true
    if(std::getline(infile, line))
    {
        sequenceSteps += 1;

        // Move the Marker in the scene and Return Transformation Matrix
        const rw::math::Transform3D<double> worldTmarker = SamplePlugin::moveMarker(markerFrame, line);

    }
    else       // End of the sequence
    {
        _timer->stop();
        _btn_Start->setText("Start Visual Sevoing");
        log().info() << "Sequence movement finished.\n";
        if (sim_dT_running)
        {
            writeImCorErrors << deltaT/1000 << ", " << maxEucD;
            for(size_t i = 0; i < max_dU_Image.size(); i += 2)
                writeImCorErrors << ", " << max_dU_Image[0+i] << ", " << max_dU_Image[1+i];
            //writeImCorErrors << deltaT/1000 << ", " << ", " << maxEucD << ", " << max_dU_Image[0] << ", " << max_dU_Image[1] << std::endl;
            writeImCorErrors << ", " << computationTimeTotal/sequenceSteps << ", " << imgRecognitionTimeTotal/sequenceSteps;
            writeImCorErrors << std::endl;
            deltaT -= 1;
            SamplePlugin::sim_dTs();
        }
        return;
    }

    // Simulate tracking with option to use image recognition
    //-----------------------------------------------------------------------------
	if (_framegrabber != NULL) {
		// Get the image as a RW image
		//Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
        Mat imFlipLabel;

        // Vector for storing actual image coordinates of target points
        std::vector<rw::math::Vector2D<int>> targetRealPixels;

        // variable for storing time needed for image recognition function
        ulong imgRec_msec = 0;

        // Check if image recognition is enabled
        if (useVision)
        {
            Mat imflip;
            cv::flip(im, imflip, -1);

            // measure time for image recognition function
            // --------------------------------------------------------
            //auto imgRec_start_time = std::chrono::high_resolution_clock::now();

            auto start = std::chrono::steady_clock::now();
            // Get target points from image
            targetRealPixels = marker1(imflip, No);
            //auto imgRec_end_time = std::chrono::high_resolution_clock::now();
            //auto imgRec_time = imgRec_end_time - imgRec_start_time;
            //imgRec_msec = std::chrono::duration_cast<std::chrono::milliseconds>(imgRec_time).count();
            // Store total image recognition time in order to compute mean time for the whole sequence

            auto finish = std::chrono::steady_clock::now();
            imgRec_msec = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count();
            imgRecognitionTimeTotal += imgRec_msec;
            log().info() << "Duration of image recognition[ms]: " << imgRec_msec << std::endl;
            // --------------------------------------------------------

            // Show Image with targetPixels in Qlabel
            // Flip image to fit into Qlabel
            cv::flip(imflip, imFlipLabel, 1);
        } else
        {
            // Simulate camera using homogeneous transforms
            targetRealPixels = SamplePlugin::cameraSimulation();

            // Flip image to fit into Qlabel
            cv::flip(im, imFlipLabel, 0);
        }

        // Show in QLabel
        QImage img(imFlipLabel.data, imFlipLabel.cols, imFlipLabel.rows, imFlipLabel.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

        // Compute joint update dQ and perform saturation to satisfy velocity constarints
        // -----------------------------------------------------------------------------------------------
        // Save start time of computations
        //auto comp_start_time = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::steady_clock::now();
        write_dU << t << ", ";
        auto dQ = SamplePlugin::algorithm2(targetRealPixels);
        auto finish = std::chrono::steady_clock::now();
        // Compute Final time of inverse kinematics
        auto time = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start).count();
        //auto comp_end_time = std::chrono::high_resolution_clock::now();
        //auto time = comp_end_time - comp_start_time;
        // save time duration of inverse kinematics algorithm
        ulong comp_msec = time;     // std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
        log().info() << "Duration of inverse kinematics[ms]: " << comp_msec << std::endl;
        // Store total inverse kinematics time in order to compute mean time for the whole sequence
        computationTimeTotal += comp_msec;

        ulong comp_and_vision_msec = comp_msec + imgRec_msec;
        log().info() << "Duration of inverse kinematics and Img rec[ms]: " << comp_and_vision_msec << std::endl;
        rw::math::Q q = device->getQ(_state);
        double movementT = 0;
        if (comp_and_vision_msec < deltaT)
        {
            // get the time left for manipulator movement
            movementT = deltaT - comp_and_vision_msec;
            log().info() << "Time for manipulator movement[ms]: " << movementT << std::endl;
            //log().info() << "dQ/movementT: " << dQ / (static_cast<double>(movementT) / 1000) << "\n";
            log().info() << "dQ: " << dQ << "\n";
            dQ = saturateDQ(dQ, velocity_limits, movementT);
            log().info() << "saturated dQ: " << dQ << "\n";
            log().info() << movementT << std::endl;
            // get new q configuration
            q += dQ;
        } else
            log().info() << "Time needed for inverse kinematics is longer than deltaT.\n";

        // For simulation of single deltaT -> save joint variables and tool position into a file
        if (!sim_dT_running)
        {
            // Save joint configuretion
            writeJoints << t << ", " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[4] << ", " << q[5]
                        << ", " << q[6] << ", " << comp_msec << ", " << imgRec_msec << ", " << movementT << std::endl;
            writeJointsVelocities << t << ", " << dQ[0]/(movementT/1000) << ", " << dQ[1]/(movementT/1000) << ", " << dQ[2]/(movementT/1000) << ", "
                                  << dQ[3]/(movementT/1000) << ", " << dQ[4]/(movementT/1000) << ", " << dQ[5]/(movementT/1000) << ", " << dQ[6]/(movementT/1000) << std::endl;

            auto worldTcamera = cameraFrame->wTf(_state);
            auto camPosition = worldTcamera.P();
            auto camOrientation = rw::math::RPY<double>(worldTcamera.R());

            // Save tool position and orientation
            writeToolPose << t << ", " << camPosition[0] << ", " << camPosition[1] << ", " << camPosition[2] << ", "
                          << camOrientation[0] << ", " << camOrientation[1] << ", " << camOrientation[2] << ", "
                          << std::endl;

            // Increase time varieble for the next loop
            t = t + static_cast<double>(deltaT) / 1000;
        }

        // update manipulator joint variables
        device->setQ(q, _state);

        // update RWStudio visualization
        getRobWorkStudio()->setState(_state);

	}
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}


//Q_EXPORT_PLUGIN(SamplePlugin);

// Workaround or fix when using qt5 for compilation
// - copied from SamplePlugin in the example dir of RWStudio -> doesn't function properly
#if !RWS_USE_QT5
#include <QtCore/qplugin.h>

#endif
Q_EXPORT_PLUGIN(SamplePlugin);
