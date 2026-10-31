cmake_minimum_required(VERSION 3.25)
project(PCLProject)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(etc)
include_directories(Jama)

set(VTK_DIR "C:/vtk/bin")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(PCLProject
        etc/2dvector.cpp
        etc/2dvector.h
        etc/matrix.cpp
        etc/matrix.h
        etc/matrix.inl
        etc/vector.cpp
        etc/vector.h
        etc/vector.inl
        Jama/jama_cholesky.cpp
        Jama/jama_cholesky.h
        Jama/jama_eig.cpp
        Jama/jama_eig.h
        Jama/jama_lu.cpp
        Jama/jama_lu.h
        Jama/jama_qr.cpp
        Jama/jama_qr.h
        Jama/jama_svd.cpp
        Jama/jama_svd.h
        Jama/tnt.h
        Jama/tnt_array1d.cpp
        Jama/tnt_array1d.h
        Jama/tnt_array1d_utils.h
        Jama/tnt_array2d.cpp
        Jama/tnt_array2d.h
        Jama/tnt_array2d_utils.h
        Jama/tnt_array3d.cpp
        Jama/tnt_array3d.h
        Jama/tnt_array3d_utils.h
        Jama/tnt_cmat.cpp
        Jama/tnt_cmat.h
        Jama/tnt_fortran_array1d.cpp
        Jama/tnt_fortran_array1d.h
        Jama/tnt_fortran_array1d_utils.h
        Jama/tnt_fortran_array2d.cpp
        Jama/tnt_fortran_array2d.h
        Jama/tnt_fortran_array2d_utils.h
        Jama/tnt_fortran_array3d.cpp
        Jama/tnt_fortran_array3d.h
        Jama/tnt_fortran_array3d_utils.h
        Jama/tnt_i_refvec.cpp
        Jama/tnt_i_refvec.h
        Jama/tnt_math_utils.h
        Jama/tnt_sparse_matrix_csr.cpp
        Jama/tnt_sparse_matrix_csr.h
        Jama/tnt_stopwatch.cpp
        Jama/tnt_stopwatch.h
        Jama/tnt_subscript.cpp
        Jama/tnt_subscript.h
        Jama/tnt_vec.cpp
        Jama/tnt_vec.h
        Jama/tnt_version.h
        DataConverstionForm.cpp
        DataConverstionForm.h
        DataConvertor.cpp
        DataConvertor.h
        ExportOptionsForm.cpp
        ExportOptionsForm.h
        ExtractionParameterForm.cpp
        ExtractionParameterForm.h
        HashMap.cpp
        HashMap.h
        InitialLocationForm.cpp
        InitialLocationForm.h
        LASFileReader.cpp
        LASFileReader.h
        LIDARPT.cpp
        LIDARPT.h
        Main.cpp
        MainForm.cpp
        MainForm.h
        PointXYZDouble.cpp
        PointXYZDouble.h
        RailwayExtraction.cpp
        RailwayExtraction.h
        SharedData.cpp
        SharedData.h
        SharedResources.cpp
        SharedResources.h
        TXTFileReader.cpp
        TXTFileReader.h
        typedef1.cpp
        typedef1.h
        UTM.cpp
        UTM.h
        Viewer.cpp
        Viewer.h
        VMouseInteractorStyle.cpp
        VMouseInteractorStyle.h
        VTKWindowControl.cpp
        VTKWindowControl.h
        VTKWindowController.cpp
        VTKWindowController.h)

target_link_libraries(PCLProject ${VTK_LIBRARIES})
