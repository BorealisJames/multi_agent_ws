/**
 * @file data_utils.h
 * @brief Provide a few widely used function for basic type
 */
#ifndef JPS_DATA_UTIL_H
#define JPS_DATA_UTIL_H

#include <jps_basis/data_type.h>

///Template for transforming a vector
template <class T, class TF>
vec_E<T> transform_vec_(const vec_E<T> &t, const TF &tf) {
  vec_E<T> new_t;
  for (const auto &it : t)
    new_t.push_back(tf * it);
  return new_t;
}

///Template for calculating distance
template <class T>
decimal_t total_distance_(const vec_E<T>& vs){
  decimal_t dist = 0;
  for(unsigned int i = 1; i < vs.size(); i++)
    dist += (vs[i] - vs[i-1]).norm();

  return dist;
}

///Transform all entries in a vector using given TF
#define transform_vec_3_ transform_vec_<Vec3f, Aff3f>
///Sum up total distance for vec_Vec2f
#define total_distance_2f_ total_distance_<Vec2f>
///Sum up total distance for vec_Vec3f
#define total_distance_3f_ total_distance_<Vec3f>
///Sum up total distance for vec_Vec2i
#define total_distance_2i_ total_distance_<Vec2i>
///Sum up total distance for vec_Vec3i
#define total_distance_3i_ total_distance_<Vec3i>
#endif
