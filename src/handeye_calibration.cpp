#include <iterator>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "json.hpp"
#include "ceres/ceres.h"
#include "ceres/types.h"
#include "camodocal/calib/HandEyeCalibration.h"

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
    eigenVector;

typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
    EigenAffineVector;

///////////////////////////////////////////////////////
// DEFINING GLOBAL VARIABLES

std::string ARTagTFname, cameraTFname;
std::string EETFname, baseTFname;
Eigen::Affine3d firstEEInverse, firstCamInverse;
bool firstTransform = true;
eigenVector tvecsArm, rvecsArm, tvecsFiducial, rvecsFiducial;

EigenAffineVector baseToTip, cameraToTag;

template <typename Input> Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat) 
{
    Eigen::AngleAxisd ax3d(eigenQuat);
    return ax3d.angle() * ax3d.axis();
}

/// @return 0 on success, otherwise error code
int readTransformPairsFromFile(	std::string filename,
								EigenAffineVector& t1,
								EigenAffineVector& t2)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    int frameCount;

    fs["frameCount"] >> frameCount;
    auto t1_it = std::back_inserter(t1);
    auto t2_it = std::back_inserter(t2);

    if (fs.isOpened()) {

        for (int i = 0; i < frameCount; ++i, ++t1_it, ++t2_it)
		{
            // read in frame one
            {
                cv::Mat_<double> t1cv = cv::Mat_<double>::ones(4, 4);
                std::stringstream ss1;
                ss1 << "T1_" << i;
                fs[ss1.str()] >> t1cv;
                Eigen::Affine3d t1e;
                cv::cv2eigen(t1cv, t1e.matrix());
                *t1_it = t1e;
            }

            // read in frame two
            {
                cv::Mat_<double> t2cv = cv::Mat_<double>::ones(4, 4);
                std::stringstream ss2;
                ss2 << "T2_" << i;
                fs[ss2.str()] >> t2cv;
                Eigen::Affine3d t2e;
                cv::cv2eigen(t2cv, t2e.matrix());
                *t2_it = t2e;
            }
        }
        fs.release();
    } 
	else
	{
        std::cerr << "failed to open input file " << filename << "\n";
        return 1;
    }
    return 0;
}

Eigen::Affine3d estimateHandEye(const EigenAffineVector& baseToTip,
                                const EigenAffineVector& camToTag) 
{

    auto t1_it = baseToTip.begin();
    auto t2_it = camToTag.begin();

    Eigen::Affine3d firstEEInverse, firstCamInverse;
    eigenVector tvecsArm, rvecsArm, tvecsFiducial, rvecsFiducial;

    bool firstTransform = true;

    for (int i = 0; i < baseToTip.size(); ++i, ++t1_it, ++t2_it) 
	{
        auto& eigenEE = *t1_it;
        auto& eigenCam = *t2_it;
        if (firstTransform)
		{
            firstEEInverse = eigenEE.inverse();
            firstCamInverse = eigenCam.inverse();
            std::cout << "Adding first transformation." << "\n";
            firstTransform = false;
        } 
		else 
		{
            Eigen::Affine3d robotTipinFirstTipBase = firstEEInverse * eigenEE;
            Eigen::Affine3d fiducialInFirstFiducialBase = firstCamInverse * eigenCam;

            rvecsArm.push_back(eigenRotToEigenVector3dAngleAxis(robotTipinFirstTipBase.rotation()));
            tvecsArm.push_back(robotTipinFirstTipBase.translation());

            rvecsFiducial.push_back(eigenRotToEigenVector3dAngleAxis( fiducialInFirstFiducialBase.rotation()));
            tvecsFiducial.push_back(fiducialInFirstFiducialBase.translation());
			std::cout << "Hand Eye Calibration Transform Pair Added." << "\n";

            Eigen::Vector4d r_tmp = robotTipinFirstTipBase.matrix().col(3);
            r_tmp[3] = 0;
            Eigen::Vector4d c_tmp = fiducialInFirstFiducialBase.matrix().col(3);
            c_tmp[3] = 0;

            std::cerr
                << "L2Norm EE: "
                << robotTipinFirstTipBase.matrix().block(0, 3, 3, 1).norm()
                << " vs Cam:"
                << fiducialInFirstFiducialBase.matrix().block(0, 3, 3, 1).norm()
                << std::endl;
        }
        std::cerr << "EE transform: \n" << eigenEE.matrix() << std::endl;
        std::cerr << "Cam transform: \n" << eigenCam.matrix() << std::endl;
    }

    camodocal::HandEyeCalibration calib;
    Eigen::Matrix4d result;
    calib.estimateHandEyeScrew(rvecsArm, tvecsArm, rvecsFiducial, tvecsFiducial,
                               result, false);
    std::cerr << "Result from " << EETFname << " to " << ARTagTFname << ":\n"
              << result << std::endl;
    Eigen::Transform<double, 3, Eigen::Affine> resultAffine(result);
    std::cerr << "Translation (x,y,z) : "
              << resultAffine.translation().transpose() << std::endl;
    Eigen::Quaternion<double> quaternionResult(resultAffine.rotation());
    std::stringstream ss;
    ss << quaternionResult.w() << ", " << quaternionResult.x() << ", "
       << quaternionResult.y() << ", " << quaternionResult.z() << std::endl;
    std::cerr << "Rotation (w,x,y,z): " << ss.str() << std::endl;

    std::cerr << "Result from " << ARTagTFname << " to " << EETFname << ":\n"
              << result << std::endl;
    Eigen::Transform<double, 3, Eigen::Affine> resultAffineInv =
        resultAffine.inverse();
    std::cerr << "Inverted translation (x,y,z) : "
              << resultAffineInv.translation().transpose() << std::endl;
    quaternionResult = Eigen::Quaternion<double>(resultAffineInv.rotation());
    ss.clear();
    ss << quaternionResult.w() << " " << quaternionResult.x() << " "
       << quaternionResult.y() << " " << quaternionResult.z() << std::endl;
    std::cerr << "Inverted rotation (w,x,y,z): " << ss.str() << std::endl;
    return resultAffine;
}

Eigen::Affine3d estimateHandEye(const EigenAffineVector& baseToTip,
                                const EigenAffineVector& camToTag,
                                ceres::Solver::Summary& summary) 
{
    auto t1_it = baseToTip.begin();
    auto t2_it = camToTag.begin();

    Eigen::Affine3d firstEEInverse, firstCamInverse;
    eigenVector tvecsArm, rvecsArm, tvecsFiducial, rvecsFiducial;

    bool firstTransform = true;

    for (int i = 0; i < baseToTip.size(); ++i, ++t1_it, ++t2_it) 
	{
        auto& eigenEE = *t1_it;
        auto& eigenCam = *t2_it;
        if (firstTransform) 
		{
            firstEEInverse = eigenEE.inverse();
            firstCamInverse = eigenCam.inverse();
			std::cout << "Adding first transformation." << "\n";
       
            firstTransform = false;
        } 
		else 
		{
            Eigen::Affine3d robotTipinFirstTipBase = firstEEInverse * eigenEE;
            Eigen::Affine3d fiducialInFirstFiducialBase = firstCamInverse * eigenCam;

            rvecsArm.push_back(eigenRotToEigenVector3dAngleAxis(robotTipinFirstTipBase.rotation()));
            tvecsArm.push_back(robotTipinFirstTipBase.translation());

            rvecsFiducial.push_back(eigenRotToEigenVector3dAngleAxis(fiducialInFirstFiducialBase.rotation()));
            tvecsFiducial.push_back(fiducialInFirstFiducialBase.translation());
			std::cout << "Hand Eye Calibration Transform Pair Added." << "\n";

            Eigen::Vector4d r_tmp = robotTipinFirstTipBase.matrix().col(3);
            r_tmp[3] = 0;
            Eigen::Vector4d c_tmp = fiducialInFirstFiducialBase.matrix().col(3);
            c_tmp[3] = 0;

            std::cerr
                << "L2Norm EE: "
                << robotTipinFirstTipBase.matrix().block(0, 3, 3, 1).norm()
                << " vs Cam:"
                << fiducialInFirstFiducialBase.matrix().block(0, 3, 3, 1).norm()
                << std::endl;
        }
        std::cerr << "EE transform: \n" << eigenEE.matrix() << std::endl;
        std::cerr << "Cam transform: \n" << eigenCam.matrix() << std::endl;
    }

    camodocal::HandEyeCalibration calib;
    Eigen::Matrix4d result;
    calib.estimateHandEyeScrew(rvecsArm, tvecsArm, rvecsFiducial, tvecsFiducial, result, summary, false);
    std::cerr << "Result from " << EETFname << " to " << ARTagTFname << ":\n" << result << std::endl;
    Eigen::Transform<double, 3, Eigen::Affine> resultAffine(result);
    std::cerr << "Translation (x,y,z) : " << resultAffine.translation().transpose() << std::endl;
    Eigen::Quaternion<double> quaternionResult(resultAffine.rotation());
    std::stringstream ss;
    ss << quaternionResult.w() << ", " << quaternionResult.x() << ", "
       << quaternionResult.y() << ", " << quaternionResult.z() << std::endl;
    std::cerr << "Rotation (w,x,y,z): " << ss.str() << std::endl;

    std::cerr << "Result from " << ARTagTFname << " to " << EETFname << ":\n"  << result << std::endl;
    Eigen::Transform<double, 3, Eigen::Affine> resultAffineInv =
        resultAffine.inverse();
    std::cerr << "Inverted translation (x,y,z) : " << resultAffineInv.translation().transpose() << std::endl;
    quaternionResult = Eigen::Quaternion<double>(resultAffineInv.rotation());
    ss.clear();
    ss << quaternionResult.w() << " " << quaternionResult.x() << " "
       << quaternionResult.y() << " " << quaternionResult.z() << std::endl;
    std::cerr << "Inverted rotation (w,x,y,z): " << ss.str() << std::endl;
    return resultAffine;
}

void writeCalibration(const Eigen::Affine3d& result,
                      const std::string& filename) 
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    std::cerr << "Writing calibration to \"" << filename << "\"...\n";
    if (fs.isOpened()) 
	{
        cv::Mat_<double> t1cv = cv::Mat_<double>::ones(4, 4);
        cv::eigen2cv(result.matrix(), t1cv);

        fs << "eeTeye" << t1cv;

        fs.release();
    }
}

void writeCalibration(const Eigen::Affine3d& result,
                      const std::string& filename,
                      ceres::Solver::Summary& summary) 
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);

    std::cerr << "FULL CONVERGENCE REPORT \""
              << "\"...\n";
    std::cout << summary.BriefReport() << "\n";
    std::cout << summary.termination_type << "\n";
    std::cerr << "Writing calibration to \"" << filename << "\"...\n";
    if (fs.isOpened()) {
        cv::Mat_<double> t1cv = cv::Mat_<double>::ones(4, 4);
        cv::eigen2cv(result.matrix(), t1cv);

        fs << "eeTeye" << t1cv;
        fs << "initial_cost" << summary.initial_cost;
        fs << "final_cost" << summary.final_cost;
        fs << "change_cost" << summary.initial_cost - summary.final_cost;
        fs << "termination_type" << summary.termination_type;
        fs << "num_successful_iteration" << summary.num_successful_steps;
        fs << "num_unsuccessful_iteration" << summary.num_unsuccessful_steps;
        fs << "num_iteration" << summary.num_unsuccessful_steps + summary.num_successful_steps;
        fs.release();
    }
}

Eigen::Affine3d estimateHandEye(const EigenAffineVector& baseToTip,
                                const EigenAffineVector& camToTag,
                                const std::string& filename,
                                const bool addSolverSummary) 
{
    if (addSolverSummary)
	{
        ceres::Solver::Summary summary;
        auto result = estimateHandEye(baseToTip, camToTag, summary);
        writeCalibration(result, filename, summary);
        return result;
    } 
	else 
	{
        auto result = estimateHandEye(baseToTip, camToTag);
        writeCalibration(result, filename);
        return result;
    }
}

int main(int argc, char** argv)
{
    std::string transformPairsLoadFile;
    std::string calibratedTransformFile;
    bool addSolverSummary = false;

	/****** read config ********************/
	std::cout << "Loading cfg..." << "\n";
	try
	{
		using json = nlohmann::json;
		json j2;
		std::ifstream i("./config.json");
		i >> j2;

		std::string param;
		param = "transformPairsLoadFile";
		if (j2.find(param) != j2.end())
			transformPairsLoadFile = j2[param].get<std::string>();
		param = "calibratedTransformFile";
		if (j2.find(param) != j2.end())
			calibratedTransformFile = j2[param].get<std::string>();
		param = "addSolverSummary";
		if (j2.find(param) != j2.end())
			addSolverSummary = j2[param].get<int>();
	}
	catch (std::exception& e)
	{
		std::cout << "exception reading parameter.\n";
		std::cout << e.what();
		return -1;
	}

	std::cout << " ==== read parameter =====" << "\n";
    std::cout << "Transform pairs loading file: " << transformPairsLoadFile << "\n";
	std::cout << "calibratedTransformFile out: " << calibratedTransformFile << "\n";
	std::cout << "addSolverSummary out: " << addSolverSummary << "\n";

    EigenAffineVector t1, t2;
    readTransformPairsFromFile(transformPairsLoadFile, t1, t2);
    auto result = estimateHandEye(t1, t2, calibratedTransformFile, addSolverSummary);

	std::cout << "================\n";
	std::cout << "press enter to continue.\n";
	std::cout << getchar();

    return 0;
}
