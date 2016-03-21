/** @file UI.h
 *
 * PaCMan UI definitions
 *
 */

#pragma once
#ifndef _PACMAN_BHAM_ACTIVE_SENSE_UI_H_ // if #pragma once is not supported
#define _PACMAN_BHAM_ACTIVE_SENSE_UI_H_

#include <Golem/UI/UI.h>

/** PaCMan name space */
namespace pacman {

namespace utils {


golem::RGBA heightMapColor(double h) {

    golem::RGBA color;

    color._rgba.a = 255;

    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:

        color._rgba.r = v*255; color._rgba.g = n*255; color._rgba.b = m*255;
        break;
    case 1:
        color._rgba.r = n*255; color._rgba.g = v*255; color._rgba.b = m*255;
        break;
    case 2:
        color._rgba.r = m*255; color._rgba.g = v*255; color._rgba.b = n*255;
        break;
    case 3:
        color._rgba.r = m*255; color._rgba.g = n*255; color._rgba.b = v*255;
        break;
    case 4:
        color._rgba.r = n*255; color._rgba.g = m*255; color._rgba.b = v*255;
        break;
    case 5:
        color._rgba.r = v*255; color._rgba.g = m*255; color._rgba.b = n*255;
        break;
    default:
        color._rgba.r = 255; color._rgba.g = 0.5*255; color._rgba.b = 0.5*255;
        break;
    }

    return color;
}






} // namespace conver
};

#endif // _PACMAN_BHAM_ACTIVE_SENSE_UI_H_
