//
// Copyeright 2017 nextEDGE Technology K.K. All Rights Reserved.
//
#ifndef _ZDCOLORPALETTE
#define _ZDCOLORPALETTE
#include <math.h>

typedef struct _RGBQUAD
{
	char rgbBlue;
	char rgbGreen;
	char rgbRed;
	char rgbReserved;
} RGBQUAD;

class ZDColotPalette
{
public:
	RGBQUAD	ColorPalette[16384];
	ZDColotPalette() {
		BuildColorPalette();
	};
	~ZDColotPalette() {}

	RGBQUAD	*GetColorPalette()
	{
		return ColorPalette;
	}
private:
	void HSV_to_RGB(double H, double S, double V, double &R, double &G, double &B)
	{
		double nMax, nMin;
		double fDet;
		//
		while (H<0.0) H += 360.0;
		while (H >= 360.0) H -= 360.0;
		H /= 60.0;
		if (V<0.0) V = 0.0;
		if (V>1.0) V = 1.0;
		V *= 255.0;
		if (S<0.0) S = 0.0;
		if (S>1.0) S = 1.0;
		//
		if (V == 0.0) {
			R = G = B = 0;
		}
		else {
			fDet = S*V;
			nMax = (V);
			nMin = (V - fDet);
			if (H <= 1.0) { //R>=G>=B, H=(G-B)/fDet
				R = nMax;
				B = nMin;
				G = (H*fDet + B);
			}
			else if (H <= 2.0) { //G>=R>=B, H=2+(B-R)/fDet
				G = nMax;
				B = nMin;
				R = ((2.0 - H)*fDet + B);
			}
			else if (H <= 3.0) { //G>=B>=R, H=2+(B-R)/fDet
				G = nMax;
				R = nMin;
				B = ((H - 2.0)*fDet + R);
			}
			else if (H <= 4.0) { //B>=G>=R, H=4+(R-G)/fDet
				B = nMax;
				R = nMin;
				G = ((4.0 - H)*fDet + R);
			}
			else if (H <= 5.0) { //B>=R>=G, H=4+(R-G)/fDet
				B = nMax;
				G = nMin;
				R = ((H - 4.0)*fDet + G);
			}
			else { // if(H<6.0) //R>=B>=G, H=(G-B)/fDet+6
				R = nMax;
				G = nMin;
				B = ((6.0 - H)*fDet + G);
			}
		}
	}
	void BuildColorPalette()
	{
		int i;
		double R, G, B;
		double fx, fy;
		//
		double fCV = 180;
		int nCenter = 1500;
		double r1 = 0.35;
		double r2 = 0.55;
		//
		for (i = 1; i<16384; i++) {
			if (i == nCenter) {
				fy = fCV;
			}
			else if (i<nCenter) {
				fx = (double)(nCenter - i) / nCenter;
				fy = fCV - pow(fx, r1)*fCV;
			}
			else {
				fx = (double)(i - nCenter) / (16384 - nCenter);
				fy = fCV + pow(fx, r2)*(256 - fCV);
			}
			HSV_to_RGB(fy, 1.0, 1.0, R, G, B);
			ColorPalette[i].rgbBlue = (BYTE)B;
			ColorPalette[i].rgbGreen = (BYTE)G;
			ColorPalette[i].rgbRed = (BYTE)R;
			ColorPalette[i].rgbReserved = 0;
		}
		{
			i = 0;
			ColorPalette[i].rgbBlue = (BYTE)128;
			ColorPalette[i].rgbGreen = (BYTE)128;
			ColorPalette[i].rgbRed = (BYTE)128;
			ColorPalette[i].rgbReserved = 0;
		}
		{
			i = 16383;
			ColorPalette[i].rgbBlue = (BYTE)255;
			ColorPalette[i].rgbGreen = (BYTE)255;
			ColorPalette[i].rgbRed = (BYTE)255;
			ColorPalette[i].rgbReserved = 0;
		}
	}
};
#endif
