/**
 * @file
 *
 * @section LICENSE
 * MIT License (MIT)
 *
 * Copyright (c) 2011 VectorNav Technologies, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * @section DESCRIPTION
 * This header file defines the error codes used within the VectorNav C/C++
 * Library.
 */
#ifndef _VN_ERRORCODES_H_
#define _VN_ERRORCODES_H_

typedef int VN_ERROR_CODE;

#define VNERR_NO_ERROR				0
#define VNERR_UNKNOWN_ERROR			1
#define VNERR_NOT_IMPLEMENTED		2
#define VNERR_TIMEOUT				3
#define VNERR_INVALID_VALUE			4
#define VNERR_FILE_NOT_FOUND		5
#define VNERR_NOT_CONNECTED			6

#endif /* _VN_ERRORCODES_H_ */
