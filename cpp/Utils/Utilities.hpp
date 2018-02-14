#ifndef UTILITIES
#define UTILITIES

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <Configuration.h>
#include <list>
#include <Eigen/Dense>

namespace myUtils
{
    // =============
    // Linear Algebra
    // =============

    // ============
    // Matrix Utils
    // ============
    Eigen::MatrixXd hStack(const Eigen::MatrixXd & a_, const Eigen::MatrixXd & b_);
    Eigen::MatrixXd vStack(const Eigen::MatrixXd & a_, const Eigen::MatrixXd & b_);
    Eigen::MatrixXd vStack(const Eigen::VectorXd & a_, const Eigen::VectorXd & b_);
    Eigen::MatrixXd deleteRow(const Eigen::MatrixXd & a_, int row);
    Eigen::MatrixXd deleteCol(const Eigen::MatrixXd & a_, int col);

    // ===========
    // Save Vector
    // ===========

    void saveVector(const Eigen::VectorXd & vec_,
                    std::string name_,
                    bool b_param = false);
    void cleaningFile(std::string file_name_,
                      std::string & ret_file_,
                      bool b_param);
    static std::list< std::string > gs_fileName_string; //global & static
} /* myUtils */

#endif
