"""Literal double-precision UTM.cpp and DataConvertor.cpp arithmetic.

Source constants, expression order, units and initialized IEEE results are kept.
These are explicit legacy conversions, not an automatic CRS inference service.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from operator import index

import numpy as np


UTM_PI = 3.14159265358979
CONVERTER_PI = 3.1415926535897932
SM_A = 6378137.0
SM_B = 6356752.314
UTM_SCALE_FACTOR = 0.9996


class UndefinedCoordinateArithmetic(ValueError):
    def __init__(self, status):
        self.status = status
        super().__init__(status)


def _int32(value):
    value = index(value)
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError("source zone parameter must fit signed int32")
    return value


def _div(a, b):
    try:
        return a / b
    except ZeroDivisionError:
        with np.errstate(divide="ignore", invalid="ignore"):
            return float(np.divide(a, b))


def _unary(function, value):
    try:
        return function(value)
    except ValueError:
        return float("nan")


def _sin(x):
    return _unary(math.sin, x)


def _cos(x):
    return _unary(math.cos, x)


def _tan(x):
    return _unary(math.tan, x)


def _sqrt(x):
    return _unary(math.sqrt, x)


def _pow(x, power):
    try:
        return math.pow(x, power)
    except (ValueError, OverflowError):
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            return float(np.power(float(x), float(power)))


def deg_to_rad(degrees: float) -> float:
    return float(degrees) / 180.0 * UTM_PI


def rad_to_deg(radians: float) -> float:
    return float(radians) / UTM_PI * 180.0


def arc_length_of_meridian(phi: float) -> float:
    phi = float(phi)
    n = (SM_A - SM_B) / (SM_A + SM_B)
    alpha = ((SM_A + SM_B) / 2.0) * (1.0 + _pow(n, 2.0) / 4.0 + _pow(n, 4.0) / 64.0)
    beta = -3.0 * n / 2.0 + 9.0 * _pow(n, 3.0) / 16.0 + -3.0 * _pow(n, 5.0) / 32.0
    gamma = 15.0 * _pow(n, 2.0) / 16.0 + -15.0 * _pow(n, 4.0) / 32.0
    delta = -35.0 * _pow(n, 3.0) / 48.0 + 105.0 * _pow(n, 5.0) / 256.0
    epsilon = 315.0 * _pow(n, 4.0) / 512.0
    return alpha * (
        phi
        + beta * _sin(2.0 * phi)
        + gamma * _sin(4.0 * phi)
        + delta * _sin(6.0 * phi)
        + epsilon * _sin(8.0 * phi)
    )


def utm_central_meridian(zone: int) -> float:
    return deg_to_rad(-183.0 + float(_int32(zone)) * 6.0)


def footpoint_latitude(y: float) -> float:
    n = (SM_A - SM_B) / (SM_A + SM_B)
    alpha = ((SM_A + SM_B) / 2.0) * (1 + _pow(n, 2.0) / 4 + _pow(n, 4.0) / 64)
    y = float(y) / alpha
    beta = 3.0 * n / 2.0 + -27.0 * _pow(n, 3.0) / 32.0 + 269.0 * _pow(n, 5.0) / 512.0
    gamma = 21.0 * _pow(n, 2.0) / 16.0 + -55.0 * _pow(n, 4.0) / 32.0
    delta = 151.0 * _pow(n, 3.0) / 96.0 + -417.0 * _pow(n, 5.0) / 128.0
    epsilon = 1097.0 * _pow(n, 4.0) / 512.0
    return (
        y
        + beta * _sin(2.0 * y)
        + gamma * _sin(4.0 * y)
        + delta * _sin(6.0 * y)
        + epsilon * _sin(8.0 * y)
    )


def map_lat_lon_to_xy(phi: float, longitude: float, central_meridian: float) -> tuple[float, float]:
    """Transverse Mercator, with all angular inputs in radians."""
    phi, longitude, central_meridian = map(float, (phi, longitude, central_meridian))
    ep2 = (_pow(SM_A, 2.0) - _pow(SM_B, 2.0)) / _pow(SM_B, 2.0)
    nu2 = ep2 * _pow(_cos(phi), 2.0)
    normal = _div(_pow(SM_A, 2.0), SM_B * _sqrt(1 + nu2))
    tangent = _tan(phi)
    t2 = tangent * tangent
    delta = longitude - central_meridian
    l3 = 1.0 - t2 + nu2
    l4 = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2)
    l5 = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2
    l6 = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2
    l7 = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2)
    l8 = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2)
    x = (
        normal * _cos(phi) * delta
        + (normal / 6.0 * _pow(_cos(phi), 3.0) * l3 * _pow(delta, 3.0))
        + (normal / 120.0 * _pow(_cos(phi), 5.0) * l5 * _pow(delta, 5.0))
        + (normal / 5040.0 * _pow(_cos(phi), 7.0) * l7 * _pow(delta, 7.0))
    )
    y = (
        arc_length_of_meridian(phi)
        + (tangent / 2.0 * normal * _pow(_cos(phi), 2.0) * _pow(delta, 2.0))
        + (tangent / 24.0 * normal * _pow(_cos(phi), 4.0) * l4 * _pow(delta, 4.0))
        + (tangent / 720.0 * normal * _pow(_cos(phi), 6.0) * l6 * _pow(delta, 6.0))
        + (tangent / 40320.0 * normal * _pow(_cos(phi), 8.0) * l8 * _pow(delta, 8.0))
    )
    return x, y


def map_xy_to_lat_lon(x: float, y: float, central_meridian: float) -> tuple[float, float]:
    """Inverse Transverse Mercator; returns latitude/longitude in radians."""
    x, y, central_meridian = map(float, (x, y, central_meridian))
    phif = footpoint_latitude(y)
    ep2 = (_pow(SM_A, 2.0) - _pow(SM_B, 2.0)) / _pow(SM_B, 2.0)
    cf = _cos(phif)
    nuf2 = ep2 * _pow(cf, 2.0)
    nf = _div(_pow(SM_A, 2.0), SM_B * _sqrt(1 + nuf2))
    nf_power = nf
    tf = _tan(phif)
    tf2, tf4 = tf * tf, (tf * tf) * (tf * tf)
    x1 = _div(1.0, nf_power * cf)
    nf_power *= nf
    x2 = _div(tf, 2.0 * nf_power)
    nf_power *= nf
    x3 = _div(1.0, 6.0 * nf_power * cf)
    nf_power *= nf
    x4 = _div(tf, 24.0 * nf_power)
    nf_power *= nf
    x5 = _div(1.0, 120.0 * nf_power * cf)
    nf_power *= nf
    x6 = _div(tf, 720.0 * nf_power)
    nf_power *= nf
    x7 = _div(1.0, 5040.0 * nf_power * cf)
    nf_power *= nf
    x8 = _div(tf, 40320.0 * nf_power)
    p2 = -1.0 - nuf2
    p3 = -1.0 - 2 * tf2 - nuf2
    p4 = (
        5.0
        + 3.0 * tf2
        + 6.0 * nuf2
        - 6.0 * tf2 * nuf2
        - 3.0 * (nuf2 * nuf2)
        - 9.0 * tf2 * (nuf2 * nuf2)
    )
    p5 = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2
    p6 = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2
    p7 = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2)
    p8 = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2)
    phi = (
        phif
        + x2 * p2 * (x * x)
        + x4 * p4 * _pow(x, 4.0)
        + x6 * p6 * _pow(x, 6.0)
        + x8 * p8 * _pow(x, 8.0)
    )
    longitude = (
        central_meridian
        + x1 * x
        + x3 * p3 * _pow(x, 3.0)
        + x5 * p5 * _pow(x, 5.0)
        + x7 * p7 * _pow(x, 7.0)
    )
    return phi, longitude


def lat_lon_to_utm_xy(
    latitude_degrees: float, longitude_degrees: float, zone: int
) -> tuple[int, float, float]:
    """Actual source inputs are DEGREES, despite UTM.h's radian comment.

    Invalid requested zones trigger the literal longitude-based cast. Its result
    is not clamped: longitude +180 can produce zone 61. No latitude normalization.
    """
    zone = _int32(zone)
    lat, lon = float(latitude_degrees), float(longitude_degrees)
    if zone < 1 or zone > 60:
        quotient = (lon + 180.0) / 6
        if not math.isfinite(quotient):
            raise UndefinedCoordinateArithmetic("undefined_source_auto_zone_cast")
        candidate = float(math.floor(quotient)) + 1
        if not -(1 << 31) <= candidate < (1 << 31):
            raise UndefinedCoordinateArithmetic("undefined_source_auto_zone_cast")
        zone = int(candidate)
    x, y = map_lat_lon_to_xy(deg_to_rad(lat), deg_to_rad(lon), utm_central_meridian(zone))
    x = x * UTM_SCALE_FACTOR + 500000.0
    y = y * UTM_SCALE_FACTOR
    if y < 0.0:
        y = y + 10000000.0
    return zone, x, y


def utm_xy_to_lat_lon(x: float, y: float, zone: int, south_hemi: bool) -> tuple[float, float]:
    """Source inverse returns RADIANS and uses the explicitly supplied hemisphere."""
    x = float(x) - 500000.0
    x /= UTM_SCALE_FACTOR
    y = float(y)
    if south_hemi:
        y -= 10000000.0
    y /= UTM_SCALE_FACTOR
    return map_xy_to_lat_lon(x, y, utm_central_meridian(zone))


@dataclass
class ConverterEllipsoid:
    a: float = 6378137.0
    e2: float = 6.6943799901377997e-3
    a1: float = 4.2697672707157535e4
    a2: float = 1.8230912546075455e9
    a3: float = 1.4291722289812413e2
    a4: float = 4.5577281365188637e9
    a5: float = 4.2840589930055659e4
    a6: float = 9.9330562000986220e-1


def ecef_to_geodetic(
    x: float, y: float, z: float, *, ellipsoid: ConverterEllipsoid | None = None
) -> tuple[float, float, float]:
    """DataConvertor's two-branch inverse, radians/radians/height; no origin repair."""
    model = ellipsoid if ellipsoid is not None else ConverterEllipsoid()
    x, y, z = map(float, (x, y, z))
    zp = abs(z)
    w2 = x * x + y * y
    w = _sqrt(w2)
    r2 = w2 + z * z
    radius = _sqrt(r2)
    lon = math.atan2(y, x)
    s2 = _div(z * z, r2)
    c2 = _div(w2, r2)
    u = _div(model.a2, radius)
    v = model.a3 - _div(model.a4, radius)
    if c2 > 0.3:
        s = _div(zp, radius) * (1.0 + _div(c2 * (model.a1 + u + s2 * v), radius))
        lat = _unary(math.asin, s)
        ss = s * s
        c = _sqrt(1.0 - ss)
    else:
        c = _div(w, radius) * (1.0 - _div(s2 * (model.a5 - u - c2 * v), radius))
        lat = _unary(math.acos, c)
        ss = 1.0 - c * c
        s = _sqrt(ss)
    g = 1.0 - model.e2 * ss
    rg = _div(model.a, _sqrt(g))
    rf = model.a6 * rg
    u = w - rg * c
    v = zp - rf * s
    f = c * u + s * v
    m = c * v - s * u
    p = _div(m, _div(rf, g) + f)
    lat = lat + p
    altitude = f + m * p / 2.0
    if z < 0.0:
        lat *= -1.0
    return lat, lon, altitude


def geodetic_to_ecef(
    latitude_radians: float,
    longitude_radians: float,
    altitude: float,
    *,
    ellipsoid: ConverterEllipsoid | None = None,
) -> tuple[float, float, float]:
    model = ellipsoid if ellipsoid is not None else ConverterEllipsoid()
    lat, lon, altitude = map(float, (latitude_radians, longitude_radians, altitude))
    normal = _div(model.a, _sqrt(1 - model.e2 * _sin(lat) * _sin(lat)))
    return (
        (normal + altitude) * _cos(lat) * _cos(lon),
        (normal + altitude) * _cos(lat) * _sin(lon),
        (normal * (1 - model.e2) + altitude) * _sin(lat),
    )
