cmake_minimum_required(VERSION 3.8)

project(sm)

add_definitions(
	-DNO_TIMER
	-DANSI_DECLARATORS
	-DTRILIBRARY
)

################################################################################
# Source groups
################################################################################
set(algorithms
    "SM_ConvexHull.cpp"
    "SM_ConvexHull.h"
    "SM_CosineSmooth.cpp"
    "SM_CosineSmooth.h"
    "SM_DouglasPeucker.cpp"
    "SM_DouglasPeucker.h"
    "SM_Polygon.cpp"
    "SM_Polygon.h"
    "SM_Polyline.cpp"
    "SM_Polyline.h"
    "SM_Triangulation.cpp"
    "SM_Triangulation.h"
)
source_group("algorithms" FILES ${algorithms})

set(complex__bounding
    "SM_Cube.h"
    "SM_Cube.inl"
    "SM_Rect.h"
    "SM_Rect.inl"
)
source_group("complex/bounding" FILES ${complex__bounding})

set(complex__plane
    "SM_Plane.h"
    "SM_Plane.inl"
)
source_group("complex/plane" FILES ${complex__plane})

set(complex__ray
    "SM_Ray.h"
    "SM_RayIntersect.cpp"
    "SM_RayIntersect.h"
)
source_group("complex/ray" FILES ${complex__ray})

set(external__CavalierContours
    "external/CavalierContours/include/cavc/intrcircle2circle2.hpp"
    "external/CavalierContours/include/cavc/intrlineseg2circle2.hpp"
    "external/CavalierContours/include/cavc/intrlineseg2lineseg2.hpp"
    "external/CavalierContours/include/cavc/mathutils.hpp"
    "external/CavalierContours/include/cavc/plinesegment.hpp"
    "external/CavalierContours/include/cavc/polyline.hpp"
    "external/CavalierContours/include/cavc/polylinecombine.hpp"
    "external/CavalierContours/include/cavc/polylineintersects.hpp"
    "external/CavalierContours/include/cavc/polylineoffset.hpp"
    "external/CavalierContours/include/cavc/polylineoffsetislands.hpp"
    "external/CavalierContours/include/cavc/staticspatialindex.hpp"
    "external/CavalierContours/include/cavc/vector.hpp"
    "external/CavalierContours/include/cavc/vector2.hpp"
)
source_group("external/CavalierContours" FILES ${external__CavalierContours})

set(external__clipper
    "external/clipper/clipper/clipper.cpp"
    "external/clipper/clipper/clipper.hpp"
    "external/clipper/clipper/clipper_export.hpp"
)
source_group("external/clipper" FILES ${external__clipper})

set(external__triangle
    "external/triangle/triangle.cpp"
    "external/triangle/triangle.h"
)
source_group("external/triangle" FILES ${external__triangle})

set(math
    "sm_c_calc.cpp"
    "sm_c_calc.h"
    "SM_Calc.cpp"
    "SM_Calc.h"
    "SM_Calc.inl"
    "SM_Ease.h"
    "SM_Math.cpp"
    "SM_Math.h"
    "SM_Math.inl"
    "SM_Process.h"
    "SM_Process.inl"
    "SM_Test.cpp"
    "SM_Test.h"
    "SM_Test.inl"
)
source_group("math" FILES ${math})

set(matrix
    "sm_c_matrix.cpp"
    "sm_c_matrix.h"
    "SM_Matrix.h"
    "SM_Matrix.inl"
    "SM_Matrix2D.cpp"
    "SM_Matrix2D.h"
    "SM_Matrix2D.inl"
)
source_group("matrix" FILES ${matrix})

set(quaternion
    "SM_Quaternion.h"
    "SM_Quaternion.inl"
)
source_group("quaternion" FILES ${quaternion})

set(utility
    "sm_const.h"
)
source_group("utility" FILES ${utility})

set(vector
    "sm_c_vector.cpp"
    "sm_c_vector.h"
    "SM_Vector.h"
    "SM_Vector.inl"
)
source_group("vector" FILES ${vector})

set(ALL_FILES
    ${algorithms}
    ${complex__bounding}
    ${complex__plane}
    ${complex__ray}
    ${external__CavalierContours}
    ${external__clipper}
    ${external__triangle}
    ${math}
    ${matrix}
    ${quaternion}
    ${utility}
    ${vector}
)

add_subdirectory(external/CavalierContours)
add_subdirectory(external/clipper)
add_subdirectory(external/triangle)

# include header dirs
include_directories(${CMAKE_SOURCE_DIR}/external)
include_directories(${CMAKE_SOURCE_DIR}/external/clipper)
include_directories(${CMAKE_SOURCE_DIR}/external/CavalierContours/include)

add_library(sm ${ALL_FILES})

target_link_libraries(sm PRIVATE clipper_static)