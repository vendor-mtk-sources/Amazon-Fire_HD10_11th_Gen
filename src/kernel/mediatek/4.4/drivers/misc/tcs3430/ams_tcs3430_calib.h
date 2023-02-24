/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/
/*
 * TCS3430 Calibration
 */
#ifndef __AMS_TCS3430_CALIB_H__
#define __AMS_TCS3430_CALIB_H__
#ifdef CONFIG_AMS_ALS_XYZ_FIXED_POINT
//#define Q_FACTOR               8
//#define K_VALUE_DEFAULT        (1 << (Q_FACTOR));
//#define HIGH_IR_THRESH_DEFAULT (1 << ((Q_FACTOR) - 1));
//#define LEFT_EDGE_DEFAULT      26;
//#define RIGHT_EDGE_DEFAULT     230;
//#define NOMINAL_ATIME_DEFAULT  25600;
//#define NOMINAL_AGAIN_DEFAULT  4096;
///* XYZ Fixed Point Calibration Matrix */
//int high_ir[3][4] = {
//    { 718, -42, -64, -24},
//    { 160,  355, -43, -23},
//    {-707,  906,  809, -26}
//};
//int low_ir[3][4] = {
//    { 474,  3, -84, -7},
//    {-113,  669, -161, -33},
//    { 14, -42,  1354, -56}
//};
#define Q_FACTOR               12
#define K_VALUE_DEFAULT        (1 << (Q_FACTOR));
#define HIGH_IR_THRESH_DEFAULT (1 << ((Q_FACTOR) - 1));
#define LEFT_EDGE_DEFAULT      410;
#define RIGHT_EDGE_DEFAULT     3686;
#define NOMINAL_ATIME_DEFAULT  409600;
#define NOMINAL_AGAIN_DEFAULT  65536;
/* XYZ Fixed Point Calibration Matrix */
int high_ir[3][4] = {
	{ 8536, -676, -1020, -385},
	{ 2564,  5673, -692, -365},
	{-11317,  14496,  12952, -414}
};
int low_ir[3][4] = {
	{ 7586,  41, -1352, -106},
	{-1815,  10699, -2568, -520},
	{ 229, -668,  21664, -897}
};
#else
#define K_VALUE_DEFAULT 1.0;
#define HIGH_IR_THRESH_DEFAULT 0.5;
#define LEFT_EDGE_DEFAULT 0.1;
#define RIGHT_EDGE_DEFAULT 0.9;
#define NOMINAL_ATIME_DEFAULT 100
#define NOMINAL_AGAIN_DEFAULT 16
/* XYZ Floating Point Calibration Matrix */
float high_ir[3][4] = {
	{ 2.084, -0.165, -0.249, -0.094},
	{ 0.626,  1.385, -0.169, -0.089},
	{-2.763,  3.539,  3.162, -0.101}
};
float low_ir[3][4] = {
	{ 1.852,  0.010, -0.330, -0.026},
	{-0.443,  2.612, -0.627, -0.127},
	{ 0.056, -0.163,  5.289, -0.219}
};
#endif
#endif  /* __AMS_TCS3430_CALIB_H__ */
