//
// Created by radio on 20.10.2024.
//

#ifndef UTILS_H
#define UTILS_H

#include <vector>

#include "H5Cpp.h"

void write_results_to_hdf5(H5::Group& channel_group, std::string& dataset_name, const std::vector<long long>& data)
{
    if (channel_group.exists(dataset_name))
    {
        channel_group.unlink(dataset_name);
    }
    // Get the size of the vector
    hsize_t vector_size = data.size();
    H5::DataSpace dataspace(1, &vector_size); // 1-D dataspace
    auto dataset_psnr = channel_group.createDataSet(dataset_name, H5::PredType::NATIVE_LLONG, dataspace);
    dataset_psnr.write(data.data(), H5::PredType::NATIVE_LLONG);
}

void write_results_to_hdf5(H5::Group& channel_group, std::string& dataset_name, const std::vector<double>& data)
{
    if (channel_group.exists(dataset_name))
    {
        channel_group.unlink(dataset_name);
    }
    // Get the size of the vector
    hsize_t vector_size = data.size();
    H5::DataSpace dataspace(1, &vector_size); // 1-D dataspace
    auto dataset_psnr = channel_group.createDataSet(dataset_name, H5::PredType::NATIVE_DOUBLE, dataspace);
    dataset_psnr.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

#endif //UTILS_H
