#include <zeno/zeno.h>
#include <OpenSim/OpenSim.h>

#include <filesystem>
#include <sstream>
#include <cstdlib>

namespace zeno
{
    struct OSTModel : zeno::IObject
    {
        OpenSim::Model _model;

        OSTModel()
            : _model{} {}

        OSTModel(const std::string &model_file)
            : _model{model_file} {}
    };

    struct MakeOSTModel : zeno::INode
    {
        virtual void apply() override
        {
            auto model_file{get_param<std::string>("model_file")};
            std::shared_ptr<OSTModel> model;
            if (model_file != "")
            {
                model = std::make_shared<OSTModel>(model_file);
            }
            else
            {
                model = std::make_shared<OSTModel>();
            }
            model->_model.initSystem();
            set_output("model", std::move(model));
        }
    };

    ZENDEFNODE(
        MakeOSTModel,
        {
            {},
            {
                {"OSTModel", "model"},
            },
            {
                {"string", "model_file", ""},
            },
            {
                "FEM",
            },
        });

    struct RunOSTIKTool : zeno::INode
    {
        virtual void apply() override
        {
            auto work_dir{get_param<std::string>("work_dir")};
            auto IKTool_file{get_param<std::string>("IKTool_file")};
            auto results_directory{get_param<std::string>("results_directory")};
            auto model_file{get_param<std::string>("model_file")};
            auto time_range{get_param<std::string>("time_range")};
            auto output_motion_file{get_param<std::string>("output_motion_file")};
            auto constraint_weight{get_param<std::string>("constraint_weight")};
            auto accuracy{get_param<std::string>("accuracy")};
            auto report_errors{get_param<std::string>("report_errors")};
            auto marker_file{get_param<std::string>("marker_file")};
            auto coordinate_file{get_param<std::string>("coordinate_file")};
            auto IKTaskSet_file{get_param<std::string>("IKTaskSet_file")};
            auto report_marker_locations{get_param<std::string>("report_marker_locations")};

            std::filesystem::path backup_path;
            if (work_dir != "")
            {
                backup_path = std::filesystem::current_path();
                std::filesystem::current_path(std::filesystem::path{work_dir});
            }

            std::shared_ptr<OpenSim::InverseKinematicsTool> tool;
            if (IKTool_file != "")
            {
                tool = std::make_shared<OpenSim::InverseKinematicsTool>(IKTool_file);
            }
            else
            {
                tool = std::make_shared<OpenSim::InverseKinematicsTool>();
            }

            if (results_directory != "")
            {
                tool->setResultsDir(results_directory);
            }

            if (model_file != "")
            {
                OpenSim::Model model{model_file};
                model.initSystem();
                tool->setModel(model);
                // tool->set_model_file(model_file);
            }

            if (time_range != "")
            {
                std::istringstream iss{time_range};
                std::string time{};

                iss >> time;
                tool->setStartTime(std::atof(time.c_str()));
                iss >> time;
                tool->setEndTime(std::atof(time.c_str()));

                // int index{0};
                // while (!iss.eof())
                // {
                //    iss >> time;
                //    tool->set_time_range(index++, std::atof(time.c_str()));
                // }
            }

            if (output_motion_file != "")
            {
                tool->setOutputMotionFileName(output_motion_file);
            }

            if (constraint_weight != "")
            {
                tool->set_constraint_weight(std::atof(constraint_weight.c_str()));
            }

            if (accuracy != "")
            {
                tool->set_accuracy(std::atof(accuracy.c_str()));
            }

            if (report_errors == "true")
            {
                tool->set_report_errors(true);
            }
            else if (report_errors == "false")
            {
                tool->set_report_errors(false);
            }

            if (marker_file != "")
            {
                tool->setMarkerDataFileName(marker_file);
            }

            if (coordinate_file != "")
            {
                tool->setCoordinateFileName(coordinate_file);
            }

            if (IKTaskSet_file != "")
            {
                tool->set_IKTaskSet(OpenSim::IKTaskSet(IKTaskSet_file));
            }

            if (report_marker_locations == "true")
            {
                tool->set_report_marker_locations(true);
            }
            else if (report_marker_locations == "false")
            {
                tool->set_report_marker_locations(false);
            }

            tool->run();
            std::cout << "IKTool run:" << '\n'
                      << "work_dir: " << std::filesystem::current_path() << '\n'
                      << "IKTool_file: " << IKTool_file << '\n'
                      << "results_directory: " << tool->getResultsDir() << '\n'
                      << "model_file: " << tool->get_model_file() << '\n'
                      << "time_range: " << tool->getStartTime() << ' ' << tool->getEndTime() << "\n"
                      << "output_motion_file: " << tool->getOutputMotionFileName() << '\n'
                      << "constraint_weight: " << tool->get_constraint_weight() << '\n'
                      << "accuracy: " << tool->get_accuracy() << '\n'
                      << "report_errors: " << tool->get_report_errors() << '\n'
                      << "marker_file: " << tool->getMarkerDataFileName() << '\n'
                      << "coordinate_file: " << tool->getCoordinateFileName() << '\n'
                      // There is no get-function
                      << "IKTaskSet_file: " << IKTaskSet_file << '\n'
                      << "report_marker_locations: " << tool->get_report_marker_locations() << '\n';

            if (work_dir != "")
            {
                std::filesystem::current_path(std::move(backup_path));
            }
        }
    }; // struct RunOSTIKTool

    ZENDEFNODE(
        RunOSTIKTool,
        {
            {},
            {},
            {
                {"string", "work_dir", "assets/opensim/"},
                {"string", "IKTool_file", "subject01_Setup_InverseKinematics.xml"},
                {"string", "results_directory", ""},
                {"string", "model_file", ""},
                // {"string", "model_file", "subject01_simbody.osim"},
                // How to code a list of double in zeno?
                {"string", "time_range", ""},
                // {"string", "time_range", "0.4 1.6"},
                {"string", "output_motion_file", ""},
                // {"string", "output_motion_file", "subject01_walk1_ik_test.mot"},
                // How to judge a null-value to double in zeno?
                {"string", "constraint_weight", ""},
                // {"string", "constraint_weight", "10.0"},
                {"string", "accuracy", ""},
                // {"string", "accuracy", "1e-5"},
                // How to judge a null-value to bool in zeno?
                {"string", "report_errors", ""},
                // {"string", "report_errors", "false"},
                {"string", "marker_file", ""},
                // {"string", "marker_file", "subject01_synthetic_marker_data.trc"},
                {"string", "coordinate_file", ""},
                {"string", "IKTaskSet_file", ""},
                // {"string", "IKTaskSet_file", "gait2354_IK_Tasks_uniform.xml"},
                {"string", "report_marker_locations", ""},
                // {"string", "report_marker_locations", "true"},
            },
            {
                "FEM",
            },
        });

    struct RunOSTIDTool : zeno::INode
    {
        virtual void apply() override
        {
            auto work_dir{get_param<std::string>("work_dir")};
            auto IDTool_file{get_param<std::string>("IDTool_file")};
            auto results_directory{get_param<std::string>("results_directory")};
            auto model_file{get_param<std::string>("model_file")};
            auto time_range{get_param<std::string>("time_range")};
            auto external_loads_file{get_param<std::string>("external_loads_file")};
            auto forces_to_exclude{get_param<std::string>("forces_to_exclude")};
            auto output_gen_force_file{get_param<std::string>("output_gen_force_file")};
            auto coordinates_file{get_param<std::string>("coordinates_file")};
            auto lowpass_cutoff_frequency_for_coordinates{
                get_param<std::string>("lowpass_cutoff_frequency_for_coordinates")};
            auto coordinates_in_degrees{get_param<std::string>("coordinates_in_degrees")};

            std::filesystem::path backup_path;
            if (work_dir != "")
            {
                backup_path = std::filesystem::current_path();
                std::filesystem::current_path(std::filesystem::path{work_dir});
            }

            std::shared_ptr<OpenSim::InverseDynamicsTool> tool;
            if (IDTool_file != "")
            {
                tool = std::make_shared<OpenSim::InverseDynamicsTool>(IDTool_file);
            }
            else
            {
                tool = std::make_shared<OpenSim::InverseDynamicsTool>();
            }

            if (results_directory != "")
            {
                tool->setResultsDir(results_directory);
            }

            if (model_file != "")
            {
                tool->setModelFileName(model_file);
            }

            if (time_range != "")
            {
                std::istringstream iss{time_range};
                std::string time{};

                iss >> time;
                tool->setStartTime(std::atof(time.c_str()));
                iss >> time;
                tool->setEndTime(std::atof(time.c_str()));

                // int index{0};
                // while (!iss.eof())
                // {
                //    iss >> time;
                //    tool->set_time_range(index++, std::atof(time.c_str()));
                // }
            }

            if (external_loads_file != "")
            {
                tool->setExternalLoadsFileName(external_loads_file);
            }

            if (forces_to_exclude != "")
            {
                std::istringstream iss{forces_to_exclude};
                std::string force{};
                OpenSim::Array<std::string> forces{};
                while (!iss.eof())
                {
                    iss >> force;
                    forces.append(force);
                }
                tool->setExcludedForces(forces);
            }

            if (output_gen_force_file != "")
            {
                tool->setOutputGenForceFileName(output_gen_force_file);
            }

            if (coordinates_file != "")
            {
                tool->setCoordinatesFileName(coordinates_file);
            }

            if (lowpass_cutoff_frequency_for_coordinates != "")
            {
                tool->setLowpassCutoffFrequency(
                    std::atof(lowpass_cutoff_frequency_for_coordinates.c_str()));
            }

            if (coordinates_in_degrees == "true")
            {
                // There is no set-function
            }
            else if (coordinates_in_degrees == "false")
            {
                // There is no set-function
            }

            tool->run();
            std::cout << "IKTool run:" << '\n'
                      << "work_dir: " << std::filesystem::current_path() << '\n'
                      << "IDTool_file: " << IDTool_file << '\n'
                      << "results_directory: " << tool->getResultsDir() << '\n'
                      << "model_file: " << tool->getModelFileName() << '\n'
                      << "time_range: " << tool->getStartTime() << ' ' << tool->getEndTime() << "\n"
                      << "external_loads_file: " << tool->getExternalLoadsFileName() << '\n'
                      // There is no get-function
                      << "forces_to_exclude: " << forces_to_exclude << '\n'
                      << "output_gen_force_file: " << tool->getOutputGenForceFileName() << '\n'
                      << "coordinates_file: " << tool->getCoordinatesFileName() << '\n'
                      << "lowpass_cutoff_frequency_for_coordinates: "
                      << tool->getLowpassCutoffFrequency() << '\n';

            if (work_dir != "")
            {
                std::filesystem::current_path(std::move(backup_path));
            }
        }
    }; // struct RunOSTIDTool

    ZENDEFNODE(
        RunOSTIDTool,
        {
            {},
            {},
            {
                {"string", "work_dir", "assets/opensim/"},
                {"string", "IDTool_file", "subject01_Setup_InverseDynamics.xml"},
                {"string", "results_directory", ""},
                // {"string", "results_directory", "Results"},
                {"string", "model_file", ""},
                // {"string", "model_file", "subject01.osim"},
                {"string", "time_range", ""},
                // {"string", "time_range", "0.5 1.6"},
                {"string", "external_loads_file", ""},
                // {"string", "external_loads_file", "subject01_walk1_grf.xml"},
                // How to code a list of string in zeno?
                {"string", "forces_to_exclude", ""},
                //{"string", "forces_to_exclude", "muscles"},
                {"string", "output_gen_force_file", ""},
                // {"string", "output_gen_force_file", "subject01_InverseDynamics"},
                {"string", "coordinates_file", ""},
                // {"string", "coordinates_file", "subject01_walk1_ik.mot"},
                {"string", "lowpass_cutoff_frequency_for_coordinates", ""},
                // {"string", "lowpass_cutoff_frequency_for_coordinates", "6"},
                {"string", "coordinates_in_degrees", ""},
                // {"string", "coordinates_in_degrees", "true"},
            },
            {
                "FEM",
            },
        });

}; // namespace zeno