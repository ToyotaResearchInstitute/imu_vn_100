/**
 * \file
 *
 * \section LICENSE
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 VectorNav Technologies, LLC
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * \section DESCRIPTION
 * This header file provides access to the VectorNav Kinematics library.
 */
#ifndef _VN_KINEMATICS_H
#define _VN_KINEMATICS_H

/**
 * \brief Holds attitude data expressed in yaw, pitch, roll format.
 */
typedef struct {
	double	yaw;		/**< Yaw */
	double	pitch;		/**< Pitch */
	double	roll;		/**< Roll */
} VnYpr;

/**
 * \brief Holds attitude data expressed in quaternion format.
 */
typedef struct {
	double x;		/**< X */
	double y;		/**< Y */
	double z;		/**< Z */
	double w;		/**< W */
} VnQuaternion;

inline VnQuaternion VnQuaternionMultiply(VnQuaternion a, VnQuaternion b) {
  VnQuaternion c;
  c.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  c.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  c.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  c.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return c;
}

#endif /* _VN_KINEMATICS_H */
