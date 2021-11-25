#!/usr/bin/env python3
import os

def mkdir(path):
    if not os.path.exists(path):
        os.mkdir(path)


def find_hdf5_File(path_to_dir, dataset_model, suffix=".hdf5"):
    filenames = os.listdir(path_to_dir)
    file_list = [filename for filename in filenames if filename.endswith(suffix)]
    for f in file_list:
        if dataset_model in f:
            file_name = f
    
    return file_name
    