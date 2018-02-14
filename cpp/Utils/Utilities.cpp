#include <Configuration.h>
#include <Utilities.hpp>

namespace myUtils {
    Eigen::MatrixXd hStack(const Eigen::MatrixXd a,
            const Eigen::MatrixXd b) {

        if ( a.rows() != b.rows() ) {
            std::cout << "[hStack] Matrix Size is Wrong" << std::endl; exit(0);
        }

        Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows(), a.cols() + b.cols());
        ab << a, b;
        return ab;
    }

    Eigen::MatrixXd vStack(const Eigen::MatrixXd a,
            const Eigen::MatrixXd b) {
        if ( a.cols() != b.cols() ) {
            std::cout << "[vStack] Matrix Size is Wrong" << std::endl;
            exit(0);
        }
        Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.rows() + b.rows(), a.cols());
        ab << a, b;
        return ab;
    }

    Eigen::MatrixXd vStack(const Eigen::VectorXd a,
            const Eigen::VectorXd b) {

        if (a.size() != b.size()) {
            std::cout << "[vStack] Vector Size is Wrong" << std::endl;
            exit(0);
        }
        Eigen::MatrixXd ab = Eigen::MatrixXd::Zero(a.size(), 2);
        ab << a, b;
        return ab;
    }

    Eigen::MatrixXd deleteRow(const Eigen::MatrixXd & a_, int row_) {
        Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(a_.rows()-1, a_.cols());
        ret.block(0, 0, row_, a_.cols()) = a_.block(0, 0, row_, a_.cols());
        ret.block(row_, 0, ret.rows()-row_, a_.cols()) =
            a_.block(row_+1, 0, ret.rows()-row_, a_.cols());
        return ret;
    }

    void saveVector(const Eigen::VectorXd & vec_, std::string name_, bool b_param){
        std::string file_name;
        cleaningFile(name_, file_name, b_param);

        std::ofstream savefile(file_name.c_str(), std::ios::app);
        for (int i(0); i < vec_.rows(); ++i){
            savefile<<vec_(i)<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void cleaningFile(std::string  _file_name, std::string & _ret_file, bool b_param){
        if(b_param)
            _ret_file += THIS_COM"parameter_data/";
        else
            _ret_file += THIS_COM"ExperimentData/";

        _ret_file += _file_name;
        _ret_file += ".txt";

        std::list<std::string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
        if(gs_fileName_string.end() == iter){
            gs_fileName_string.push_back(_file_name);
            remove(_ret_file.c_str());
        }
    }
}
