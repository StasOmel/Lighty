#include "tools.h"

void toolsHSVtoRGB(ColorHSV_t *hsv, ColorRGB_t *rgb)
{
  uint32_t  f=0, H=0, S=0, V=0;
  uint32_t w=0, q=0, t=0, i=0;
  uint32_t r=0, g=0, b=0;

  H = hsv->h;
  S = hsv->s;
  V = hsv->v;

  if (S==0) { r = g = b = V; }
  else
    {
      i = H / 60;
      f= (((H*100)/60)-(i*100));
      w = V * (100 - S) / 100;
      q = V * (100 * 100 - (S * f)) / 10000;
      t = V * (100 * 100 - (S * (100 - f))) / 10000;

      switch (i)
      {
      case 0:
      case 6:
        r = V;
        g = t;
        b = w;
        break;
      case 1:
        r = q;
        g = V;
        b = w;
        break;
      case 2:
        r = w;
        g = V;
        b = t;
        break;
      case 3:
        r = w;
        g = q;
        b = V;
        break;
      case 4:
        r = t;
        g = w;
        b = V;
        break;
      case 5:
        r = V;
        g = w;
        b = q;
        break;
      }
    }

  rgb->r = (r * 255)/100;
  rgb->g = (g * 255)/100;
  rgb->b = (b * 255)/100;
}

//void toolsHSVtoRGB(ColorHSV_t *hsv, ColorRGB_t *rgb)
//{
//	uint32_t  f=0, hue=0, sat=0, val=0;
//	uint32_t p, q, t;
//
//	hue = hsv->h;
//	sat = hsv->s;
//	val = hsv->v;
//
//	if (sat==0) { rgb->r = rgb->g = rgb->b = val; }
//	else
//	{
//	    f = ((hue % 60) * 255) / 60;
//	    hue /= 60;
//	    p = val * (255 - sat) / 255;
//	    q = val * (255 - (sat * f) / 255) / 255;
//	    t = val * (255 - (sat * (255 - f)) / 255) / 255;
//	    switch (hue)
//	    {
//	        case 0: rgb->r = val; rgb->g = t; rgb->b = p; break;
//	        case 1: rgb->r = q; rgb->g = val; rgb->b = p; break;
//	        case 2: rgb->r = p; rgb->g = val; rgb->b = t; break;
//	        case 3: rgb->r = p; rgb->g = q; rgb->b = val; break;
//	        case 4: rgb->r = t; rgb->g = p; rgb->b = val; break;
//	        case 5: rgb->r = val; rgb->g = p; rgb->b = q; break;
//	    }
//	}
//}
