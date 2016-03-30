#ifdef __cplusplus
extern "C"
{
#endif

#ifndef spatial_math_test_h
#define spatial_math_test_h

#include "sm_vector.h"

#include <float.h>
#include <stddef.h>

static inline struct sm_vec3 *
sm_intersection_raytriangle(const struct sm_vec3 *rayOrig, const struct sm_vec3 *rayDir,
	const struct sm_vec3 *vert0, const struct sm_vec3 *vert1, const struct sm_vec3 *vert2,
	struct sm_vec3 *intsPoint) {
	// Idea: Tomas Moeller and Ben Trumbore
	// in Fast, Minimum Storage Ray/Triangle Intersection 
	
	// Find vectors for two edges sharing vert0
	struct sm_vec3 edge1, edge2;
	sm_vec3_vector(&edge1, vert1, vert0);
	sm_vec3_vector(&edge2, vert2, vert0);

	// Begin calculating determinant - also used to calculate U parameter
	struct sm_vec3 pvec;
	sm_vec3_cross(&pvec, rayDir, &edge2);

	// If determinant is near zero, ray lies in sm_plane of triangle
	float det = sm_vec3_dot(&edge1, &pvec);

	// *** Culling branch ***
	/*if( det < FLT_EPSILON )
		return NULL;

	// Calculate distance from vert0 to ray origin
	struct sm_vec3 tvec;
	sm_vec3_vector(&tvec, rayOrig, &vert0);

	// Calculate U parameter and test bounds
	float u = sm_vec3_dot(&tvec, &pvec);
	if (u < 0 || u > det ) 
		return NULL;

	// Prepare to test V parameter
	struct sm_vec3 qvec;
	sm_vec3_cross(&qvec, &tvec, &edge1);

	// Calculate V parameter and test bounds
	float v = sm_vec3_dot(rayDir, &qvec);
	if (v < 0 || u + v > det ) 
		return NULL;

	// Calculate t, scale parameters, ray intersects triangle
	float t = sm_vec3_dot(&edge2, &qvec ) / det;*/

	// *** Non-culling branch ***
	if( det > -FLT_EPSILON && det < FLT_EPSILON )
		return 0;
	float inv_det = 1.0f / det;

	// Calculate distance from vert0 to ray origin
	struct sm_vec3 tvec;
	sm_vec3_vector(&tvec, rayOrig, vert0);

	// Calculate U parameter and test bounds
	float u = sm_vec3_dot(&tvec, &pvec ) * inv_det;
	if( u < 0.0f || u > 1.0f ) 
		return 0;

	// Prepare to test V parameter
	struct sm_vec3 qvec;
	sm_vec3_cross(&qvec, &tvec, &edge1);

	// Calculate V parameter and test bounds
	float v = sm_vec3_dot(rayDir, &qvec ) * inv_det;
	if( v < 0.0f || u + v > 1.0f ) 
		return 0;

	// Calculate t, ray intersects triangle
	float t = sm_vec3_dot(&edge2, &qvec) * inv_det;

	// Calculate intersection point and test ray length and direction
	intsPoint->x = rayOrig->x + rayDir->x * t;
	intsPoint->y = rayOrig->y + rayDir->y * t;
	intsPoint->z = rayOrig->z + rayDir->z * t;

	struct sm_vec3 vec;
	sm_vec3_vector(&vec, intsPoint, rayOrig);
	if( sm_vec3_dot(&vec, rayDir) < 0 || sm_vec3_length(&vec) > sm_vec3_length(rayDir)) 
		return NULL;

	return intsPoint;
}

static inline float
sm_minf(float a, float b) {
	return a < b ? a : b;
}

static inline float
sm_maxf(float a, float b) {
	return a > b ? a : b;
}

static inline int
sm_intersection_rayAABB(const struct sm_vec3 *rayOrig, const struct sm_vec3 *rayDir, 
	const struct sm_vec3 *mins, const struct sm_vec3 *maxs ) {
	// SLAB based optimized ray/AABB intersection routine
	// Idea taken from http://ompf.org/ray/
	
	float l1 = (mins->x - rayOrig->x) / rayDir->x;
	float l2 = (maxs->x - rayOrig->x) / rayDir->x;
	float lmin = sm_minf( l1, l2 );
	float lmax = sm_maxf( l1, l2 );

	l1 = (mins->y - rayOrig->y) / rayDir->y;
	l2 = (maxs->y - rayOrig->y) / rayDir->y;
	lmin = sm_maxf( sm_minf( l1, l2 ), lmin );
	lmax = sm_minf( sm_maxf( l1, l2 ), lmax );
		
	l1 = (mins->z - rayOrig->z) / rayDir->z;
	l2 = (maxs->z - rayOrig->z) / rayDir->z;
	lmin = sm_maxf( sm_minf( l1, l2 ), lmin );
	lmax = sm_minf( sm_maxf( l1, l2 ), lmax );

	if( (lmax >= 0.0f) & (lmax >= lmin) ) {
		// Consider length
		const struct sm_vec3 rayDest = { rayOrig->x + rayDir->x , rayOrig->y + rayDir->y , rayOrig->z + rayDir->z };
		const struct sm_vec3 rayMins = { sm_minf( rayDest.x, rayOrig->x), sm_minf( rayDest.y, rayOrig->y ), sm_minf( rayDest.z, rayOrig->z ) };
		const struct sm_vec3 rayMaxs = { sm_maxf( rayDest.x, rayOrig->x), sm_maxf( rayDest.y, rayOrig->y ), sm_maxf( rayDest.z, rayOrig->z ) };
		return 
			(rayMins.x < maxs->x) && (rayMaxs.x > mins->x) &&
			(rayMins.y < maxs->y) && (rayMaxs.y > mins->y) &&
			(rayMins.z < maxs->z) && (rayMaxs.z > mins->z);
	} else {
		return 0;
	}
}

static inline float 
sm_vec3_distAABB(const struct sm_vec3 *pos, const struct sm_vec3 *mins, const struct sm_vec3 *maxs ) {
	struct sm_vec3 center;
	struct sm_vec3 extent;
	center.x = (mins->x + maxs->x) * 0.5f;
	center.y = (mins->y + maxs->y) * 0.5f;
	center.z = (mins->z + maxs->z) * 0.5f;

	extent.x = (maxs->x - mins->x) * 0.5f;
	extent.y = (maxs->y - mins->y) * 0.5f;
	extent.z = (maxs->z - mins->z) * 0.5f;
	
	struct sm_vec3 nearestVec;
	nearestVec.x = sm_maxf( 0, fabsf( pos->x - center.x ) - extent.x );
	nearestVec.y = sm_maxf( 0, fabsf( pos->y - center.y ) - extent.y );
	nearestVec.z = sm_maxf( 0, fabsf( pos->z - center.z ) - extent.z );
	
	return sm_vec3_length(&nearestVec);
}

#endif // spatial_math_test_h

#ifdef __cplusplus
}
#endif