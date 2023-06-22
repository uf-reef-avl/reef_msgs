//
// Created by prashant on 1/6/21.
//

#ifndef REEF_MSGS_TEST_UTILITIES_H
#define REEF_MSGS_TEST_UTILITIES_H
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
#include "json.hpp"

namespace test_utilities {
    template<typename Derived>
    void str_to_EigenMatrix(Eigen::MatrixBase<Derived> &mat, int rowSize, int columnSize, std::string str,
                            std::string delim) {
        boost::erase_all(str, " ");
        boost::erase_all(str, "[");
        boost::erase_all(str, "]");
        std::vector<std::string> strvec;
        boost::algorithm::trim_if(str, boost::algorithm::is_any_of(delim));
        boost::algorithm::split(strvec, str, boost::algorithm::is_any_of(delim), boost::algorithm::token_compress_on);
        int count = 0;
        for (int i = 0; i < rowSize; i++) {
            for (int j = 0; j < columnSize; j++) {
                mat(i, j) = boost::lexical_cast<double>(strvec[count]);
                count++;
            }
        }
    }
}

#endif //REEF_MSGS_TEST_UTILITIES_H
