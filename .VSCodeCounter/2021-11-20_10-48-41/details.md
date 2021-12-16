# Details

Date : 2021-11-20 10:48:41

Directory /home/rodrigo/Dropbox/RESEARCH/RTSLAM/SLAM_Bebop

Total : 89 files,  14284 codes, 4019 comments, 5072 blanks, all 23375 lines

[summary](results.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [Makefile](/Makefile) | Makefile | 62 | 35 | 50 | 147 |
| [README.md](/README.md) | Markdown | 29 | 0 | 28 | 57 |
| [bin/control_plan.txt](/bin/control_plan.txt) | Django txt | 6 | 0 | 5 | 11 |
| [control_plan_example.txt](/control_plan_example.txt) | Django txt | 14 | 0 | 1 | 15 |
| [index-1.html](/index-1.html) | Django HTML | 28 | 0 | 0 | 28 |
| [src/Bebop2.cpp](/src/Bebop2.cpp) | C++ | 1,454 | 648 | 442 | 2,544 |
| [src/Bebop2.h](/src/Bebop2.h) | C++ | 217 | 89 | 59 | 365 |
| [src/Fullnavdata.cpp](/src/Fullnavdata.cpp) | C++ | 225 | 23 | 91 | 339 |
| [src/Fullnavdata.h](/src/Fullnavdata.h) | C++ | 68 | 61 | 34 | 163 |
| [src/Jacs/JAC_XYZ_uvr.cpp](/src/Jacs/JAC_XYZ_uvr.cpp) | C++ | 41 | 34 | 31 | 106 |
| [src/Jacs/JacSystemPredictionV3b.cpp](/src/Jacs/JacSystemPredictionV3b.cpp) | C++ | 252 | 90 | 24 | 366 |
| [src/Jacs/JacSystemPredictionV3b.h](/src/Jacs/JacSystemPredictionV3b.h) | C++ | 7 | 18 | 5 | 30 |
| [src/Jacs/Jac_XYZ_uvr.h](/src/Jacs/Jac_XYZ_uvr.h) | C++ | 10 | 6 | 12 | 28 |
| [src/Jacs/Jac_uv_XYZ.h](/src/Jacs/Jac_uv_XYZ.h) | C++ | 10 | 6 | 13 | 29 |
| [src/Jacs/Jav_uv_XYZ.cpp](/src/Jacs/Jav_uv_XYZ.cpp) | C++ | 40 | 8 | 16 | 64 |
| [src/Transforms/AngleWrap.cpp](/src/Transforms/AngleWrap.cpp) | C++ | 16 | 0 | 3 | 19 |
| [src/Transforms/AngleWrap.h](/src/Transforms/AngleWrap.h) | C++ | 4 | 0 | 7 | 11 |
| [src/Transforms/Euler_to_Ra2b.cpp](/src/Transforms/Euler_to_Ra2b.cpp) | C++ | 41 | 41 | 18 | 100 |
| [src/Transforms/Euler_to_Ra2b.h](/src/Transforms/Euler_to_Ra2b.h) | C++ | 7 | 18 | 6 | 31 |
| [src/Transforms/Geo2ECEF.cpp](/src/Transforms/Geo2ECEF.cpp) | C++ | 21 | 4 | 14 | 39 |
| [src/Transforms/Geo2ECEF.h](/src/Transforms/Geo2ECEF.h) | C++ | 4 | 3 | 6 | 13 |
| [src/Transforms/Makefile](/src/Transforms/Makefile) | Makefile | 9 | 0 | 9 | 18 |
| [src/Transforms/Ra2b_TO_Quat_a2b.cpp](/src/Transforms/Ra2b_TO_Quat_a2b.cpp) | C++ | 11 | 24 | 6 | 41 |
| [src/Transforms/Ra2b_TO_Quat_a2b.h](/src/Transforms/Ra2b_TO_Quat_a2b.h) | C++ | 6 | 18 | 5 | 29 |
| [src/Transforms/quat2R.cpp](/src/Transforms/quat2R.cpp) | C++ | 85 | 28 | 11 | 124 |
| [src/Transforms/quat2R.h](/src/Transforms/quat2R.h) | C++ | 6 | 18 | 5 | 29 |
| [src/anms/anms.cpp](/src/anms/anms.cpp) | C++ | 262 | 7 | 54 | 323 |
| [src/anms/anms.h](/src/anms/anms.h) | C++ | 44 | 8 | 24 | 76 |
| [src/anms/nanoflann.hpp](/src/anms/nanoflann.hpp) | C++ | 816 | 401 | 182 | 1,399 |
| [src/anms/range-tree/lrtypes.h](/src/anms/range-tree/lrtypes.h) | C++ | 12 | 0 | 5 | 17 |
| [src/anms/range-tree/ranget.h](/src/anms/range-tree/ranget.h) | C++ | 485 | 62 | 166 | 713 |
| [src/control/control.cpp](/src/control/control.cpp) | C++ | 657 | 370 | 184 | 1,211 |
| [src/control/control.h](/src/control/control.h) | C++ | 61 | 6 | 40 | 107 |
| [src/ekf/altitude_update.cpp](/src/ekf/altitude_update.cpp) | C++ | 66 | 6 | 34 | 106 |
| [src/ekf/altitude_update.h](/src/ekf/altitude_update.h) | C++ | 8 | 4 | 8 | 20 |
| [src/ekf/attitude_update.cpp](/src/ekf/attitude_update.cpp) | C++ | 63 | 309 | 27 | 399 |
| [src/ekf/attitude_update.h](/src/ekf/attitude_update.h) | C++ | 12 | 6 | 9 | 27 |
| [src/ekf/cl_position_update.cpp](/src/ekf/cl_position_update.cpp) | C++ | 39 | 5 | 31 | 75 |
| [src/ekf/cl_position_update.h](/src/ekf/cl_position_update.h) | C++ | 8 | 4 | 11 | 23 |
| [src/ekf/ekf.cpp](/src/ekf/ekf.cpp) | C++ | 89 | 6 | 44 | 139 |
| [src/ekf/ekf.h](/src/ekf/ekf.h) | C++ | 67 | 0 | 42 | 109 |
| [src/ekf/ekf_types.h](/src/ekf/ekf_types.h) | C++ | 70 | 5 | 20 | 95 |
| [src/ekf/prediction.cpp](/src/ekf/prediction.cpp) | C++ | 59 | 51 | 36 | 146 |
| [src/ekf/prediction.h](/src/ekf/prediction.h) | C++ | 8 | 4 | 10 | 22 |
| [src/ekf/speed_update.cpp](/src/ekf/speed_update.cpp) | C++ | 43 | 18 | 18 | 79 |
| [src/ekf/speed_update.h](/src/ekf/speed_update.h) | C++ | 8 | 4 | 9 | 21 |
| [src/ekf/store.cpp](/src/ekf/store.cpp) | C++ | 123 | 7 | 50 | 180 |
| [src/ekf/store.h](/src/ekf/store.h) | C++ | 13 | 0 | 14 | 27 |
| [src/ekf/system_init.cpp](/src/ekf/system_init.cpp) | C++ | 55 | 17 | 23 | 95 |
| [src/ekf/system_init.h](/src/ekf/system_init.h) | C++ | 11 | 4 | 11 | 26 |
| [src/ekf/visual_delete_feats.cpp](/src/ekf/visual_delete_feats.cpp) | C++ | 48 | 9 | 41 | 98 |
| [src/ekf/visual_delete_feats.h](/src/ekf/visual_delete_feats.h) | C++ | 11 | 4 | 11 | 26 |
| [src/ekf/visual_init_anchors.cpp](/src/ekf/visual_init_anchors.cpp) | C++ | 67 | 7 | 52 | 126 |
| [src/ekf/visual_init_anchors.h](/src/ekf/visual_init_anchors.h) | C++ | 12 | 4 | 8 | 24 |
| [src/ekf/visual_init_w_range.cpp](/src/ekf/visual_init_w_range.cpp) | C++ | 160 | 83 | 82 | 325 |
| [src/ekf/visual_init_w_range.h](/src/ekf/visual_init_w_range.h) | C++ | 14 | 4 | 17 | 35 |
| [src/ekf/visual_match_feats.cpp](/src/ekf/visual_match_feats.cpp) | C++ | 224 | 70 | 82 | 376 |
| [src/ekf/visual_match_feats.h](/src/ekf/visual_match_feats.h) | C++ | 21 | 4 | 14 | 39 |
| [src/ekf/visual_update_f.cpp](/src/ekf/visual_update_f.cpp) | C++ | 106 | 22 | 64 | 192 |
| [src/ekf/visual_update_f.h](/src/ekf/visual_update_f.h) | C++ | 14 | 4 | 15 | 33 |
| [src/getData.cpp](/src/getData.cpp) | C++ | 162 | 21 | 54 | 237 |
| [src/getData.h](/src/getData.h) | C++ | 77 | 24 | 35 | 136 |
| [src/keyboardControlBebop2.cpp](/src/keyboardControlBebop2.cpp) | C++ | 278 | 72 | 125 | 475 |
| [src/keyboardControlBebop2.h](/src/keyboardControlBebop2.h) | C++ | 181 | 32 | 71 | 284 |
| [src/locks.h](/src/locks.h) | C++ | 15 | 0 | 22 | 37 |
| [src/loop/cost_function_cl.h](/src/loop/cost_function_cl.h) | C++ | 193 | 31 | 79 | 303 |
| [src/loop/loop.cpp](/src/loop/loop.cpp) | C++ | 465 | 294 | 179 | 938 |
| [src/loop/loop.h](/src/loop/loop.h) | C++ | 41 | 0 | 32 | 73 |
| [src/map/Add_KeyFrames.cpp](/src/map/Add_KeyFrames.cpp) | C++ | 279 | 125 | 103 | 507 |
| [src/map/Add_KeyFrames2.cpp](/src/map/Add_KeyFrames2.cpp) | C++ | 330 | 52 | 123 | 505 |
| [src/map/cost_function.h](/src/map/cost_function.h) | C++ | 172 | 48 | 66 | 286 |
| [src/map/delete_ak.cpp](/src/map/delete_ak.cpp) | C++ | 83 | 5 | 35 | 123 |
| [src/map/g_store.cpp](/src/map/g_store.cpp) | C++ | 45 | 2 | 17 | 64 |
| [src/map/g_store.h](/src/map/g_store.h) | C++ | 13 | 0 | 9 | 22 |
| [src/map/local_bundle_ajustment.cpp](/src/map/local_bundle_ajustment.cpp) | C++ | 194 | 50 | 65 | 309 |
| [src/map/local_bundle_ajustment2.cpp](/src/map/local_bundle_ajustment2.cpp) | C++ | 200 | 53 | 60 | 313 |
| [src/map/map.cpp](/src/map/map.cpp) | C++ | 69 | 8 | 27 | 104 |
| [src/map/map.h](/src/map/map.h) | C++ | 66 | 1 | 55 | 122 |
| [src/map/map_types.h](/src/map/map_types.h) | C++ | 23 | 2 | 10 | 35 |
| [src/map/visual_match.cpp](/src/map/visual_match.cpp) | C++ | 157 | 74 | 67 | 298 |
| [src/matplotlib/matplotlibcpp.h](/src/matplotlib/matplotlibcpp.h) | C++ | 2,184 | 137 | 666 | 2,987 |
| [src/matplotlib/matplotlibcpp_x.h](/src/matplotlib/matplotlibcpp_x.h) | C++ | 1,848 | 120 | 587 | 2,555 |
| [src/matplotlib/numpy_flags.py](/src/matplotlib/numpy_flags.py) | Python | 7 | 2 | 4 | 13 |
| [src/parameters.cpp](/src/parameters.cpp) | C++ | 128 | 12 | 43 | 183 |
| [src/parameters.h](/src/parameters.h) | C++ | 164 | 13 | 47 | 224 |
| [src/plot.cpp](/src/plot.cpp) | C++ | 308 | 42 | 48 | 398 |
| [src/plot.h](/src/plot.h) | C++ | 22 | 4 | 20 | 46 |
| [src/vision/vision.cpp](/src/vision/vision.cpp) | C++ | 139 | 94 | 73 | 306 |
| [src/vision/vision.h](/src/vision/vision.h) | C++ | 12 | 14 | 11 | 37 |

[summary](results.md)