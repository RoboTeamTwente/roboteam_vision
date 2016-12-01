#pragma once

namespace rtt {
    const float PI = 3.14159265359;
    const float HALF_PI = 1.57079632679;

    /**
     * Converts the milimeters used by SSL vision to meters.
     * millimeters -> meters
     */
    float mm_to_m(float scalar);
}
