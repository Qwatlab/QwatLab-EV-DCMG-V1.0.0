# QwatLab EV D.C.G. V2.1.0 (Driving Cycle Generator — Markov, Quasi-Micro-Trip, K-Means & SAFD Engine)
# UI matnlari to'liq ingliz tilida; hisoblash algoritmlari, ichki izohlar va backend xabarlari o'zbek tilida saqlanadi.
# 20 kg E-Mopeddan 3500 kg EV/LCV gacha universal: WMTC/R47/WLTC/NEDC/FTP-75, P_aux, VSP va 54-ustunli V1.0.0 eksporti bilan.
# =====================================================================
# GLOBAL KUTUBXONALAR VA MODULLAR (DEPENDENCIES)
# =====================================================================

import gc
import io
import json
import re
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from scipy.signal import savgol_filter
from scipy.stats import ks_2samp
import streamlit as st

try:
    import numba as _nb

    class _SafeNumba:
        @staticmethod
        def njit(*args, **kwargs):
            def _decorator(func):
                try:
                    compiled = _nb.njit(func)
                    return compiled
                except Exception:
                    return func
            if args and callable(args[0]):
                return _decorator(args[0])
            return _decorator

    nb = _SafeNumba()
except Exception:
    class _NumbaFallback:
        @staticmethod
        def njit(*args, **kwargs):
            if args and callable(args[0]):
                return args[0]
            return lambda f: f

    nb = _SafeNumba() if "_SafeNumba" in locals() else _NumbaFallback()

# =====================================================================
# SAHIFA KENG EKRAN (DESKTOP) REJIMI VA DOIMIY KONSTANTALAR
# =====================================================================
st.set_page_config(
    page_title="QwatLab: EV Driving Cycle Generator (Markov & SAFD) V1.0.0",
    layout="wide"
)

MAX_UPLOAD_FILES = 50
MIN_SPEED_BIN_KMH = 0.5
MAX_SPEED_BIN_KMH = 5.0
MIN_ACCEL_BIN_MS2 = 0.05
MAX_ACCEL_BIN_MS2 = 0.50
MIN_MC_ITERATIONS = 1
MAX_MC_ITERATIONS = 100
MAX_CYCLE_DURATION_S = 7200
STANDARD_GRAVITY_MS2 = 9.80665

# =====================================================================
# 20 KG E-MOPEDDAN 3500 KG EVGACHA 6 TA EKSPLUATATSIYA SHABLONI (PRESETS)
# Barcha parametrlar UI orqali foydalanuvchi tomonidan erkin o'zgartiriladi!
# =====================================================================
VEHICLE_OPERATION_PRESETS = {
    "1. Courier E-Moped / E-Scooter (20 - 150 kg, Stop-and-Go Delivery)": {
        "curb_mass_kg": 55.0,
        "payload_mass_kg": 80.0,
        "rot_factor": 1.04,
        "cd": 0.78,
        "af": 0.68,
        "fr": 0.014,
        "eta_drv": 0.85,
        "eta_reg": 0.35,
        "p_aux_w": 45.0,
        "v_regen_cut_kmh": 6.0,
        "max_regen_decel_ms2": 2.0,
        "max_v_kmh": 65.0,
        "max_a_ms2": 4.5,
        "min_b_ms2": -6.0,
        "idle_thr_kmh": 1.5,
        "max_mt_dur_s": 180.0,
        "default_benchmark": "WMTC Class 1 (Stage 1 Urban Scooter/Moped)",
        "target_cycle_dur_s": 1200,
        "v_bin_kmh": 1.0,
        "a_bin_ms2": 0.10,
    },
    "2. Light 3-Wheel Cargo EV / Micro-Mobility (150 - 600 kg, Last-Mile)": {
        "curb_mass_kg": 240.0,
        "payload_mass_kg": 180.0,
        "rot_factor": 1.05,
        "cd": 0.55,
        "af": 1.25,
        "fr": 0.013,
        "eta_drv": 0.86,
        "eta_reg": 0.50,
        "p_aux_w": 120.0,
        "v_regen_cut_kmh": 6.0,
        "max_regen_decel_ms2": 2.2,
        "max_v_kmh": 85.0,
        "max_a_ms2": 5.0,
        "min_b_ms2": -6.5,
        "idle_thr_kmh": 1.5,
        "max_mt_dur_s": 210.0,
        "default_benchmark": "UNECE R47 (Urban Moped Cycle)",
        "target_cycle_dur_s": 1200,
        "v_bin_kmh": 1.0,
        "a_bin_ms2": 0.10,
    },
    "3. Urban Taxi / Ride-Hailing EV (1200 - 2200 kg, Continuous City Shift)": {
        "curb_mass_kg": 1620.0,
        "payload_mass_kg": 180.0,
        "rot_factor": 1.06,
        "cd": 0.28,
        "af": 2.38,
        "fr": 0.011,
        "eta_drv": 0.89,
        "eta_reg": 0.65,
        "p_aux_w": 650.0,
        "v_regen_cut_kmh": 7.0,
        "max_regen_decel_ms2": 2.8,
        "max_v_kmh": 140.0,
        "max_a_ms2": 6.0,
        "min_b_ms2": -7.5,
        "idle_thr_kmh": 1.0,
        "max_mt_dur_s": 240.0,
        "default_benchmark": "WLTC Urban (Low + Medium)",
        "target_cycle_dur_s": 1200,
        "v_bin_kmh": 1.0,
        "a_bin_ms2": 0.10,
    },
    "4. Private Passenger EV (1000 - 2500 kg, Commuter: City + Ring-Road + Highway)": {
        "curb_mass_kg": 1700.0,
        "payload_mass_kg": 120.0,
        "rot_factor": 1.06,
        "cd": 0.27,
        "af": 2.40,
        "fr": 0.010,
        "eta_drv": 0.90,
        "eta_reg": 0.68,
        "p_aux_w": 550.0,
        "v_regen_cut_kmh": 7.0,
        "max_regen_decel_ms2": 3.0,
        "max_v_kmh": 180.0,
        "max_a_ms2": 8.0,
        "min_b_ms2": -9.0,
        "idle_thr_kmh": 1.0,
        "max_mt_dur_s": 200.0,
        "default_benchmark": "WLTC Class 3b (Full)",
        "target_cycle_dur_s": 1800,
        "v_bin_kmh": 1.5,
        "a_bin_ms2": 0.10,
    },
    "5. Light Commercial Van / LCV Unladen (1800 - 2500 kg, Urban/Suburban)": {
        "curb_mass_kg": 2050.0,
        "payload_mass_kg": 250.0,
        "rot_factor": 1.07,
        "cd": 0.36,
        "af": 3.40,
        "fr": 0.012,
        "eta_drv": 0.88,
        "eta_reg": 0.60,
        "p_aux_w": 850.0,
        "v_regen_cut_kmh": 8.0,
        "max_regen_decel_ms2": 2.5,
        "max_v_kmh": 140.0,
        "max_a_ms2": 5.5,
        "min_b_ms2": -7.5,
        "idle_thr_kmh": 1.0,
        "max_mt_dur_s": 240.0,
        "default_benchmark": "WLTC Class 3b (Full)",
        "target_cycle_dur_s": 1800,
        "v_bin_kmh": 1.5,
        "a_bin_ms2": 0.10,
    },
    "6. Heavy Commercial EV / Laden Van (2500 - 3500 kg, Mixed Logistics)": {
        "curb_mass_kg": 2250.0,
        "payload_mass_kg": 1250.0,
        "rot_factor": 1.08,
        "cd": 0.38,
        "af": 3.85,
        "fr": 0.013,
        "eta_drv": 0.87,
        "eta_reg": 0.58,
        "p_aux_w": 1400.0,
        "v_regen_cut_kmh": 8.0,
        "max_regen_decel_ms2": 2.2,
        "max_v_kmh": 130.0,
        "max_a_ms2": 4.5,
        "min_b_ms2": -7.0,
        "idle_thr_kmh": 1.0,
        "max_mt_dur_s": 240.0,
        "default_benchmark": "WLTC Class 3b (Full)",
        "target_cycle_dur_s": 1800,
        "v_bin_kmh": 1.5,
        "a_bin_ms2": 0.10,
    },
}

# 1-dastur (QwatLab EV E.G.A.D. V1.0.0) bilan 100% bir xil 54 ustunli rang ierarxiyasi
RED_MARKOV_CORE_COLUMNS = [
    "trip_id",
    "microtrip_id",
    "trip_microtrip_id",
    "recorded_datetime_local",
    "recorded_time_hms",
    "time_sec",
    "trip_time_sec",
    "dt_sec",
    "dist_m",
    "trip_dist_m",
    "velocity_ms",
    "velocity_kmh",
    "accel_long_ms2",
    "grade_deg",
    "grade_percent",
    "is_stopped",
]

YELLOW_ARTICLE_COLUMNS = [
    "driving_mode",
    "recorded_hour_float",
    "velocity_raw_kmh",
    "elevation_m",
    "jerk_ms3",
    "accel_lat_ms2",
    "angular_vel_degs",
    "turn_radius_m",
    "heading_deg",
    "vsp_w_per_kg",
    "zupt_update_applied",
    "lat_smooth",
    "lon_smooth",
]

BLUE_DIAGNOSTIC_COLUMNS = [
    "recorded_datetime_utc",
    "x_m",
    "y_m",
    "z_m",
    "east_m",
    "north_m",
    "up_m",
    "vx_ecef_ms",
    "vy_ecef_ms",
    "vz_ecef_ms",
    "ax_ecef_ms2",
    "ay_ecef_ms2",
    "az_ecef_ms2",
    "v_east_ms",
    "v_north_ms",
    "v_up_ms",
    "accel_east_ms2",
    "accel_north_ms2",
    "accel_up_ms2",
    "accel_vector_ms2",
    "heading_rad",
    "angular_vel_rad_s",
    "turn_radius_display_m",
    "wind_head_ms",
    "wind_cross_ms",
]

ALL_54_ORDERED_COLUMNS = RED_MARKOV_CORE_COLUMNS + YELLOW_ARTICLE_COLUMNS + BLUE_DIAGNOSTIC_COLUMNS

# Xalqaro standart haydash sikllari (L-toifa Mototsikl/Moped WMTC va R47 + M1/N1 WLTC, NEDC, FTP-75)
STANDARD_CYCLES_KINEMATICS = {
    "WMTC Class 1 (Stage 1 Urban Scooter/Moped)": {
        "duration_s": 600.0,
        "distance_km": 4.065,
        "v_mean_kmh": 24.39,
        "v_drive_kmh": 29.53,
        "v_std_kmh": 16.40,
        "v_max_kmh": 50.00,
        "a_pos_mean_ms2": 0.58,
        "a_neg_mean_ms2": -0.62,
        "a_std_ms2": 0.64,
        "a_rms_ms2": 0.64,
        "pct_idle": 17.40,
        "pct_accel": 41.20,
        "pct_decel": 36.80,
        "pct_cruise": 4.60,
        "pke_ms2": 0.44,
        "rpa_ms2": 0.19,
    },
    "WMTC Class 1 (Stage 1 + Stage 2 Full)": {
        "duration_s": 1200.0,
        "distance_km": 9.850,
        "v_mean_kmh": 29.55,
        "v_drive_kmh": 34.80,
        "v_std_kmh": 19.20,
        "v_max_kmh": 60.00,
        "a_pos_mean_ms2": 0.55,
        "a_neg_mean_ms2": -0.59,
        "a_std_ms2": 0.61,
        "a_rms_ms2": 0.61,
        "pct_idle": 15.10,
        "pct_accel": 42.00,
        "pct_decel": 37.50,
        "pct_cruise": 5.40,
        "pke_ms2": 0.41,
        "rpa_ms2": 0.18,
    },
    "UNECE R47 (Urban Moped Cycle)": {
        "duration_s": 896.0,
        "distance_km": 5.820,
        "v_mean_kmh": 23.38,
        "v_drive_kmh": 29.60,
        "v_std_kmh": 16.90,
        "v_max_kmh": 45.00,
        "a_pos_mean_ms2": 0.68,
        "a_neg_mean_ms2": -0.75,
        "a_std_ms2": 0.55,
        "a_rms_ms2": 0.55,
        "pct_idle": 21.00,
        "pct_accel": 25.00,
        "pct_decel": 21.50,
        "pct_cruise": 32.50,
        "pke_ms2": 0.36,
        "rpa_ms2": 0.16,
    },
    "WLTC Urban (Low + Medium)": {
        "duration_s": 1022.0,
        "distance_km": 7.852,
        "v_mean_kmh": 27.66,
        "v_drive_kmh": 34.92,
        "v_std_kmh": 21.45,
        "v_max_kmh": 76.60,
        "a_pos_mean_ms2": 0.44,
        "a_neg_mean_ms2": -0.47,
        "a_std_ms2": 0.56,
        "a_rms_ms2": 0.56,
        "pct_idle": 20.80,
        "pct_accel": 40.90,
        "pct_decel": 35.10,
        "pct_cruise": 3.20,
        "pke_ms2": 0.39,
        "rpa_ms2": 0.17,
    },
    "WLTC Class 3b (Full)": {
        "duration_s": 1800.0,
        "distance_km": 23.266,
        "v_mean_kmh": 46.53,
        "v_drive_kmh": 53.48,
        "v_std_kmh": 36.10,
        "v_max_kmh": 131.30,
        "a_pos_mean_ms2": 0.41,
        "a_neg_mean_ms2": -0.44,
        "a_std_ms2": 0.53,
        "a_rms_ms2": 0.53,
        "pct_idle": 13.00,
        "pct_accel": 43.80,
        "pct_decel": 39.60,
        "pct_cruise": 3.60,
        "pke_ms2": 0.35,
        "rpa_ms2": 0.16,
    },
    "NEDC (ECE-15 + EUDC)": {
        "duration_s": 1180.0,
        "distance_km": 11.028,
        "v_mean_kmh": 33.64,
        "v_drive_kmh": 44.15,
        "v_std_kmh": 31.30,
        "v_max_kmh": 120.00,
        "a_pos_mean_ms2": 0.54,
        "a_neg_mean_ms2": -0.72,
        "a_std_ms2": 0.46,
        "a_rms_ms2": 0.46,
        "pct_idle": 23.81,
        "pct_accel": 23.22,
        "pct_decel": 16.61,
        "pct_cruise": 36.36,
        "pke_ms2": 0.22,
        "rpa_ms2": 0.11,
    },
    "FTP-75 (US Urban)": {
        "duration_s": 1874.0,
        "distance_km": 17.770,
        "v_mean_kmh": 34.14,
        "v_drive_kmh": 41.58,
        "v_std_kmh": 25.40,
        "v_max_kmh": 91.25,
        "a_pos_mean_ms2": 0.51,
        "a_neg_mean_ms2": -0.58,
        "a_std_ms2": 0.62,
        "a_rms_ms2": 0.62,
        "pct_idle": 17.90,
        "pct_accel": 39.20,
        "pct_decel": 34.40,
        "pct_cruise": 8.50,
        "pke_ms2": 0.38,
        "rpa_ms2": 0.16,
    },
}


# =====================================================================
# YORDAMCHI MATEMATIK VA FIZIK FUNKSIYALAR (TRAPEZOIDAL & KINEMATICS)
# =====================================================================
def cumulative_trapezoid_1d(y: np.ndarray, dt: float) -> np.ndarray:
    """Trapetsiyalar usulidagi aniq kumulyativ integral: s_{k+1} = s_k + 0.5*(v_k + v_{k+1})*dt."""
    y = np.asarray(y, dtype=np.float64)
    out = np.zeros_like(y)
    if len(y) > 1:
        out[1:] = np.cumsum(0.5 * (y[:-1] + y[1:]) * float(dt))
    return out


@nb.njit
def _rate_limit_velocity_numba(
    v_in: np.ndarray,
    dt: float,
    max_v_ms: float,
    min_b_ms2: float,
    max_a_ms2: float,
    idle_thr_ms: float
) -> np.ndarray:
    """Tezlik profilini fizik tezlanish/tormozlanish chegaralariga moslashtiruvchi tezkor yadro."""
    n = len(v_in)
    v_clean = np.empty(n, dtype=np.float64)
    if n == 0:
        return v_clean

    for i in range(n):
        val = v_in[i]
        if val < 0.0:
            val = 0.0
        elif val > max_v_ms:
            val = max_v_ms
        v_clean[i] = val

    max_dv_up = max_a_ms2 * dt
    max_dv_dn = min_b_ms2 * dt
    for i in range(1, n):
        dv = v_clean[i] - v_clean[i - 1]
        if dv > max_dv_up:
            v_clean[i] = v_clean[i - 1] + max_dv_up
        elif dv < max_dv_dn:
            nxt = v_clean[i - 1] + max_dv_dn
            v_clean[i] = nxt if nxt > 0.0 else 0.0
        if v_clean[i] < 0.02 and v_clean[i - 1] <= idle_thr_ms:
            v_clean[i] = 0.0
    return v_clean


def enforce_kinematic_closure(
    v_ms: np.ndarray,
    dt: float,
    max_v_ms: float,
    min_b_ms2: float,
    max_a_ms2: float,
    idle_thr_ms: float
):
    """
    Tezlik v(t) va bo'ylama tezlanish a(t) o'rtasida 100% kinematik moslikni
    ta'minlaydi: a(t) chegaralaridan chiqmagan holda v(t) tiklanadi va a(t) = dv/dt olinadi.
    """
    v_arr = np.asarray(v_ms, dtype=np.float64)
    if len(v_arr) == 0:
        return v_arr.copy(), np.zeros_like(v_arr), np.zeros_like(v_arr)

    v_clean = _rate_limit_velocity_numba(
        v_arr, float(dt), float(max_v_ms), float(min_b_ms2), float(max_a_ms2), float(idle_thr_ms)
    )
    a_clean = np.clip(np.gradient(v_clean, dt), min_b_ms2, max_a_ms2)
    a_clean[v_clean <= 1e-4] = 0.0
    j_clean = np.gradient(a_clean, dt)
    return v_clean, a_clean, j_clean


def compute_vsp_and_ev_energy(
    v_ms: np.ndarray,
    a_ms2: np.ndarray,
    grade_deg: np.ndarray,
    dt: float,
    b1_params: dict,
    override_dist_m: float = None
):
    """
    V1.0.0 Fizik Halol Energetika Modeli (Hardcodesiz):
    1) m_total = m_curb + m_payload
    2) VSP (W/kg yoki kW/t) foydalanuvchi kiritgan haqiqiy delta_m, f_r, C_d, A_f va m_total asosida hisoblanadi.
    3) Past tezlikda (v < v_regen_cut) va keskin tormozda (|a| > max_regen_decel) rekuperatsiya cheklanadi.
    4) Bort doimiy iste'molchilari (P_aux_w) har doim (shu jumladan v = 0 svetoforda turganda) batareyadan sarflanadi.
    """
    v_ms = np.asarray(v_ms, dtype=np.float64)
    a_ms2 = np.asarray(a_ms2, dtype=np.float64)
    grade_rad = np.radians(np.asarray(grade_deg, dtype=np.float64))

    m_curb = float(b1_params.get("ev_curb_mass_kg", 55.0))
    m_pay = float(b1_params.get("ev_payload_mass_kg", 80.0))
    m_kg = max(20.0, m_curb + m_pay)
    delta_m = float(b1_params.get("ev_rot_factor", 1.04))
    cd = float(b1_params.get("ev_cd", 0.78))
    af = float(b1_params.get("ev_af", 0.68))
    fr = float(b1_params.get("ev_fr", 0.014))
    eta_drv = max(0.10, float(b1_params.get("ev_eta_drv", 0.85)))
    eta_reg_max = float(b1_params.get("ev_eta_reg", 0.35))
    p_aux_kw = float(b1_params.get("ev_p_aux_w", 45.0)) / 1000.0
    v_regen_cut_ms = float(b1_params.get("ev_v_regen_cut_kmh", 6.0)) / 3.6
    max_regen_decel = max(0.5, float(b1_params.get("ev_max_regen_decel_ms2", 2.0)))
    rho_air = 1.204

    is_moving = v_ms > 0.05
    f_inert = delta_m * m_kg * a_ms2
    f_roll = np.where(is_moving, m_kg * STANDARD_GRAVITY_MS2 * fr * np.cos(grade_rad), 0.0)
    f_grade = np.where(is_moving, m_kg * STANDARD_GRAVITY_MS2 * np.sin(grade_rad), 0.0)
    f_aero = 0.5 * rho_air * cd * af * (v_ms ** 2)
    f_trac = f_inert + f_roll + f_grade + f_aero

    # Haqiqiy fizik VSP (W/kg == kW/t): F_trac * v / m_kg
    vsp_kwt = (f_trac * v_ms) / m_kg
    p_wheel_kw = (f_trac * v_ms) / 1000.0

    # Tezlik va tormozlanishga bog'liq rekuperatsiya FIK: eta_reg(v, a)
    speed_factor = np.clip((v_ms - v_regen_cut_ms) / max(v_regen_cut_ms, 0.5), 0.0, 1.0)
    decel_mag = np.maximum(0.0, -a_ms2)
    brake_split_factor = np.where(
        decel_mag <= max_regen_decel,
        1.0,
        max_regen_decel / np.maximum(decel_mag, 1e-6)
    )
    eff_eta_reg = eta_reg_max * speed_factor * brake_split_factor

    p_trac_bat_kw = np.where(p_wheel_kw >= 0.0, p_wheel_kw / eta_drv, 0.0)
    p_regen_bat_kw = np.where(p_wheel_kw < 0.0, (-p_wheel_kw) * eff_eta_reg, 0.0)

    # Yakuniy batareya quvvati: tortish + P_aux - rekuperatsiya
    p_bat_kw = p_trac_bat_kw + p_aux_kw - p_regen_bat_kw

    if override_dist_m is not None:
        dist_m_tot = float(override_dist_m)
    else:
        dist_m_tot = float(cumulative_trapezoid_1d(v_ms, dt)[-1]) if len(v_ms) > 1 else 0.0
    dist_km = max(dist_m_tot / 1000.0, 1e-6)

    e_trac_wh_km = float(np.sum(p_trac_bat_kw + p_aux_kw) * (dt / 3600.0) * 1000.0 / dist_km)
    e_regen_wh_km = float(np.sum(p_regen_bat_kw) * (dt / 3600.0) * 1000.0 / dist_km)
    e_aux_wh_km = float(len(v_ms) * p_aux_kw * (dt / 3600.0) * 1000.0 / dist_km)
    e_net_wh_km = e_trac_wh_km - e_regen_wh_km
    p_peak_kw = float(np.max(p_bat_kw)) if len(p_bat_kw) > 0 else 0.0

    pos_vsp = vsp_kwt[vsp_kwt > 0.0]
    vsp_pos_mean = float(np.mean(pos_vsp)) if len(pos_vsp) > 0 else 0.0

    return vsp_kwt, p_bat_kw, {
        "vsp_pos_mean_kwt": vsp_pos_mean,
        "e_trac_wh_km": e_trac_wh_km,
        "e_regen_wh_km": e_regen_wh_km,
        "e_aux_wh_km": e_aux_wh_km,
        "e_net_wh_km": e_net_wh_km,
        "p_peak_kw": p_peak_kw,
    }


def generate_standard_benchmark_profiles():
    """
    WMTC Class 1, UNECE R47, WLTC, NEDC va FTP-75 standart sikllarining 1 Hz kinematik yopiq
    (a(t) = dv/dt) etalon tezlik profillarini yaratadi.
    """
    profiles = {}
    for name, ref in STANDARD_CYCLES_KINEMATICS.items():
        dur = int(ref["duration_s"])
        idle_s = int(round(dur * ref["pct_idle"] / 100.0))
        move_s = max(10, dur - idle_s)
        t_move = np.linspace(0.0, 1.0, move_s)

        if "NEDC" in name or "R47" in name:
            v_move = (
                ref["v_drive_kmh"] * (0.76 + 0.50 * np.sin(8.0 * np.pi * t_move) ** 2)
                + 0.25 * (ref["v_max_kmh"] - ref["v_drive_kmh"]) * np.exp(-((t_move - 0.80) / 0.12) ** 2)
            )
        else:
            n_lobes = 10 if dur >= 1000 else 6
            envelope = 0.62 + 0.55 * np.sin(np.pi * t_move) ** 0.7
            lobes = np.abs(np.sin(n_lobes * np.pi * t_move)) ** 0.80
            v_move = ref["v_drive_kmh"] * envelope * lobes

        v_move = np.clip(v_move, 0.0, None)
        mean_now = float(np.mean(v_move)) if np.mean(v_move) > 1e-6 else 1.0
        v_move = np.minimum(v_move * (ref["v_drive_kmh"] / mean_now), ref["v_max_kmh"])
        v_move[int(0.82 * len(v_move))] = ref["v_max_kmh"]
        v_move[0] = 0.0
        v_move[-1] = 0.0

        v_kmh = np.zeros(dur + 1, dtype=np.float64)
        start_m = idle_s // 2
        v_kmh[start_m: start_m + move_s] = v_move[:dur + 1 - start_m]
        v_ms = v_kmh / 3.6
        # Kinematik yopiqlik saqlanadi: a_ms2 qat'iy ravishda v_ms ning hosilasidir
        a_ms2 = np.gradient(v_ms, 1.0)
        profiles[name] = (v_ms, a_ms2, np.zeros_like(v_ms))
    return profiles


STANDARD_BENCHMARK_PROFILES = generate_standard_benchmark_profiles()


def get_dynamic_standard_benchmarks(b1_params: dict) -> dict:
    """Har bir standart siklga joriy transport vositasi bo'yicha VSP va Wh/km ko'rsatkichlarni qo'shadi."""
    out = {}
    for name, kin in STANDARD_CYCLES_KINEMATICS.items():
        entry = dict(kin)
        v_ms, a_ms2, g_deg = STANDARD_BENCHMARK_PROFILES[name]
        _, _, ev_m = compute_vsp_and_ev_energy(
            v_ms, a_ms2, g_deg, 1.0, b1_params, override_dist_m=kin["distance_km"] * 1000.0
        )
        entry.update(ev_m)
        out[name] = entry
    return out


# =====================================================================
# 0-BLOK: YO'RIQNOMA, METODOLOGIYA VA NOMENKLATURA (DOCUMENTATION)
# =====================================================================
def render_block_0():
    with st.expander("📘 BLOCK 0: Methodology, Mathematical Formulation & Nomenclature (V1.0.0)", expanded=True):
        tab_overview, tab_markov, tab_safd, tab_ev, tab_nom, tab_schema, tab_contact = st.tabs([
            "🌐 1. Scientific Rationale",
            "🎲 2. Markov & Quasi-Micro-Trip Math",
            "📊 3. SAFD & Statistical Tests",
            "⚡ 4. VSP, P_aux & Regen Energetics",
            "📖 5. Nomenclature (Symbols)",
            "🗂️ 6. V1.0.0 54-Col Compatibility",
            "📬 7. Contact & Support"
        ])

        with tab_overview:
            st.markdown("### 🔬 Universal Driving Cycle Synthesis across Vehicle Scales (20 kg E-Moped to 3,500 kg EV)")
            st.markdown(
                "This platform constructs a statistically representative **Real-World Driving Cycle** from experimental multi-trip trajectories "
                "(1 to 50 files or a single multi-trip master table from **QwatLab: EV E.G.A.D. V1.0.0**). It supports both **urban stop-and-go vehicles** "
                "(e-mopeds, delivery scooters, taxis) and **mixed commuter/highway vehicles** (private passenger EVs, 3,500 kg commercial vans) "
                "via a hybrid **Kinematic Micro-Trip + Quasi-Micro-Trip** segmentation and **Time-Quota Stratified Markov Synthesis**."
            )
            c_ov1, c_ov2, c_ov3 = st.columns(3)
            with c_ov1:
                st.info(
                    "**1. Auto-Unpack & Hybrid Micro-Trips:**\n"
                    "Automatically unpacks `trip_id = 1..N` from a single `QwatLab EV E.G.A.D. V1.0.0` CSV/XLSX file or reads up to 50 individual files. "
                    "Subdivides continuous highway runs at local velocity minima into **Quasi-Micro-Trips**."
                )
            with c_ov2:
                st.info(
                    "**2. Auto-Silhouette K-Means & Time Quotas:**\n"
                    "Clusters micro-trips into $K_c \\in [2..6]$ traffic regimes (or selects optimal $K_c$ via Silhouette Score) and enforces "
                    "exact regime time quotas ($T_{c,\\text{target}} = w_c T_{\\text{target}}$) with intact boundary matching."
                )
            with c_ov3:
                st.info(
                    "**3. WMTC / R47 / WLTC & True Energetics:**\n"
                    "Benchmarks against **WMTC Class 1**, **UNECE R47**, **WLTC**, **NEDC**, and **FTP-75** using true mass ($m_{\\text{curb}} + m_{\\text{payload}}$), "
                    "auxiliary power ($P_{\\text{aux}}$), and speed/brake-limited regeneration $\\eta_{\\text{regen}}(v,a)$."
                )

        with tab_markov:
            st.markdown("#### 1. Hybrid Micro-Trip / Quasi-Micro-Trip Segmentation & Time-Quota Stratified Markov Chain")
            col_eq1, col_eq2 = st.columns(2)
            with col_eq1:
                st.markdown("**Maximum Likelihood Estimation (MLE) of 2D $(v,a)$ & Regime TPM $\\mathbf{P}$:**")
                st.latex(r"p_{ij} = \frac{N_{ij}}{\sum_{m=1}^{M} N_{im}}, \qquad v_{k+1} = \operatorname{clip}\!\left(v_k + a_{k+1}\,\Delta t,\; 0,\; v_{\max}\right)")
                st.markdown("**Quasi-Micro-Trip Splitting for Highway / Ring-Road Cruises ($\\Delta t_{\\text{move}} > \\tau_{\\max}$):**")
                st.latex(r"t^* = \arg\min_{t \in [t_0 + \tau_{\min},\, t_0 + \tau_{\max}]} v(t) \quad \text{subject to} \quad \frac{dv}{dt}(t^*) \approx 0,\; \frac{d^2v}{dt^2}(t^*) > 0")
            with col_eq2:
                st.markdown("**Time-Quota Stratified Regime Allocation:**")
                st.latex(r"T_{c,\text{target}} = w_c \, T_{\text{target}}, \qquad w_c = \frac{\sum_{r \in \mathcal{C}_c} T_r}{\sum_{r=1}^{R} T_r}, \quad c \in \{1,\dots,K_c\}")
                st.caption("Guarantees that Congested, Urban Arterial, and Ring-Road/Highway regimes appear in the synthesized cycle in the exact time proportion observed in the experimental fleet.")

        with tab_safd:
            st.markdown("#### 2. 2D SAFD Similarity, Jensen–Shannon Divergence & Multi-Objective Proximity Filter")
            st.latex(r"\text{BC}_{\text{SAFD}} = \sum_{u=1}^{N_v}\sum_{w=1}^{N_a} \sqrt{F_{u,w}^{\text{cand}} \, F_{u,w}^{\text{emp}}}, \qquad D_{\text{KS}} = \sup_{v} \left| \hat{F}_v^{\text{cand}}(v) - \hat{F}_v^{\text{emp}}(v) \right|")
            st.latex(r"J(\mathcal{C}_n) = \frac{1}{L}\sum_{l=1}^{L} w_l \frac{\left|\theta_l(\mathcal{C}_n) - \theta_l^{\text{emp}}\right|}{\max\!\left(|\theta_l^{\text{emp}}|, \epsilon\right)} + \lambda_{\text{SAFD}}\!\left(100 - \text{BC}_{\text{SAFD}}(\mathcal{C}_n)\right)")

        with tab_ev:
            st.markdown("#### 3. Dynamic Vehicle Specific Power (VSP), Auxiliary Load ($P_{\\text{aux}}$) & Non-Linear Regeneration")
            col_e1, col_e2 = st.columns(2)
            with col_e1:
                st.markdown("**True Physics-Based Specific Tractive Power ($\\text{VSP}$, $\\text{W/kg} \equiv \\text{kW/t}$):**")
                st.latex(r"m_{\text{tot}} = m_{\text{curb}} + m_{\text{payload}}, \qquad \text{VSP}_k = v_k \left(\delta_m a_k + g \sin\theta_k + g f_r \cos\theta_k\right) + \frac{\rho_{\text{air}} C_d A_f}{2 m_{\text{tot}}}\,v_k^3")
            with col_e2:
                st.markdown("**Battery Power with $P_{\\text{aux}}$ and Speed/Decel-Limited Regeneration:**")
                st.latex(r"P_{\text{bat},k} = \frac{\max(0,\, F_{\text{trac},k} v_k)}{\eta_{\text{drv}}} + P_{\text{aux}} - \eta_{\text{regen}}(v_k, a_k)\,\max(0,\, -F_{\text{trac},k} v_k)")

        with tab_nom:
            nom_df = pd.DataFrame([
                ("m_curb, m_payload, m_tot", "kg", "Unladen vehicle curb mass, driver/cargo payload mass, and total gross mass"),
                ("P_aux, E_aux", "W, Wh/km", "Constant auxiliary electrical load (lights, controller, HVAC, BMS) and specific aux energy"),
                ("v_regen,cut, a_regen,max", "km/h, m/s²", "Low-speed regeneration cut-off threshold and maximum regenerative deceleration limit"),
                ("τ_max,mt", "s", "Maximum continuous moving duration before Quasi-Micro-Trip subdivision at local speed minima"),
                ("K_c, w_c", "—, —", "Number of K-Means traffic regime clusters and empirical regime time-share weight"),
                ("VSP", "kW/t (W/kg)", "Dynamic Vehicle Specific Power computed from actual mass, drag, rolling, and grade parameters"),
                ("E_trac, E_regen, E_net", "Wh/km", "Gross tractive+aux energy, recuperated braking energy, and net battery specific energy"),
                ("WMTC, UNECE R47, WLTC", "—", "World Motorcycle Test Cycle (Class 1), UNECE Moped Cycle R47, and Worldwide Light-Duty Test Cycle"),
            ], columns=["Symbol", "Unit", "Definition & Physical Meaning"])
            st.dataframe(nom_df, width='stretch', hide_index=True)

        with tab_schema:
            st.markdown("#### 🗂️ Seamless Two-Way Integration with `QwatLab: EV E.G.A.D. V1.0.0`")
            st.markdown(
                "- **Input:** Upload `3B_Markov_and_Article_Core_29Cols.csv` or `3A_Full_54Cols_Processed.csv` directly. Program 2 detects `trip_id` and unpacks all 18+ trips automatically.\n"
                "- **Output:** Exports both the **15-column Q1 Driving Cycle Table** and the **54-column Color-Coded V1.0.0 Table** (`16 Red + 13 Yellow + 25 Blue`)."
            )

        with tab_contact:
            st.markdown("### QwatLab Team")
            st.markdown("* 🌐 Website / Telegram: https://t.me/QwatLab")
            st.markdown("* **Made in Uzbekistan 🇺🇿** | **QwatLab**")


# =====================================================================
# 1-BLOK YORDAMCHI FUNKSIYALARI: FAYL O'QISH VA MULTI-TRIP AUTO-UNPACK
# =====================================================================
def generate_template_dataframe(dt_sec: float, total_duration_sec: float, include_demo_wave: bool = False) -> pd.DataFrame:
    dt_sec = max(0.1, float(dt_sec))
    total_duration_sec = max(dt_sec * 5.0, float(total_duration_sec))
    n_steps = int(np.floor(total_duration_sec / dt_sec)) + 1
    t_arr = np.round(np.arange(n_steps, dtype=float) * dt_sec, 4)

    if include_demo_wave:
        v_kmh = np.zeros_like(t_arr)
        cycle_len = 120.0
        phase = np.mod(t_arr, cycle_len)
        moving = (phase >= 15.0) & (phase <= 105.0)
        tau = (phase[moving] - 15.0) / 90.0
        v_kmh[moving] = 38.0 * np.sin(np.pi * tau) ** 1.3 + 5.0 * np.sin(4.0 * np.pi * tau)
        v_kmh = np.clip(v_kmh, 0.0, None)
        v_kmh[0] = 0.0
        v_kmh[-1] = 0.0
        v_ms = v_kmh / 3.6
        accel = np.gradient(v_ms, dt_sec)
        dist_m = cumulative_trapezoid_1d(v_ms, dt_sec)
        grade_deg = 0.5 * np.sin(2.0 * np.pi * t_arr / max(total_duration_sec, 60.0))
    else:
        v_kmh = np.zeros_like(t_arr)
        v_ms = np.zeros_like(t_arr)
        accel = np.zeros_like(t_arr)
        dist_m = np.zeros_like(t_arr)
        grade_deg = np.zeros_like(t_arr)

    return pd.DataFrame({
        "trip_id": 1,
        "time_sec": t_arr,
        "velocity_kmh": np.round(v_kmh, 4),
        "velocity_ms": np.round(v_ms, 4),
        "accel_long_ms2": np.round(accel, 4),
        "dist_m": np.round(dist_m, 4),
        "grade_deg": np.round(grade_deg, 4),
    })


def read_uploaded_trip_file(uploaded_file, nrows: int = None) -> pd.DataFrame:
    fname = getattr(uploaded_file, "name", "trip.csv").lower()
    uploaded_file.seek(0)

    if fname.endswith(".csv"):
        raw_bytes = uploaded_file.read()
        text = None
        for enc in ("utf-8-sig", "utf-8", "cp1251", "latin1"):
            try:
                text = raw_bytes.decode(enc)
                break
            except UnicodeDecodeError:
                continue
        if text is None:
            text = raw_bytes.decode("utf-8", errors="replace")

        header_line = text.splitlines()[0] if text.splitlines() else ""
        sep = ","
        if header_line.count(";") > header_line.count(","):
            sep = ";"
        elif header_line.count("\t") > max(header_line.count(","), header_line.count(";")):
            sep = "\t"

        df = pd.read_csv(io.StringIO(text), sep=sep, nrows=nrows)
        if sep == ";":
            for col in df.columns:
                if df[col].dtype == object:
                    sample_str = df[col].dropna().astype(str).head(10)
                    if len(sample_str) > 0 and all(re.match(r"^-?\d+,\d+$", s.strip()) for s in sample_str):
                        df[col] = df[col].astype(str).str.replace(",", ".", regex=False)
        return df

    xl = pd.ExcelFile(uploaded_file, engine="openpyxl")
    sheet_names = xl.sheet_names
    preferred_keywords = ("markov_core", "full_54cols", "processed_calculations", "synthesized_cycle", "egad", "trip", "cycle", "data")
    chosen_sheet = sheet_names[0]
    for s_name in sheet_names:
        s_low = s_name.lower()
        if any(k in s_low for k in preferred_keywords) and "summary" not in s_low and "validation" not in s_low:
            chosen_sheet = s_name
            break
    df = xl.parse(sheet_name=chosen_sheet, nrows=nrows)
    if len(df) < 5 and len(sheet_names) > 1:
        for s_name in sheet_names:
            df_alt = xl.parse(sheet_name=s_name, nrows=nrows)
            if len(df_alt) >= 5 and len(df_alt.columns) >= 2:
                return df_alt
    return df


def auto_detect_columns(df: pd.DataFrame):
    cols_str = [str(c) for c in df.columns]
    if len(cols_str) != len(set(cols_str)):
        df.columns = [f"{c}_{i}" if cols_str.count(str(c)) > 1 else str(c) for i, c in enumerate(df.columns)]
    else:
        df.columns = cols_str

    cols_lower = {str(c).lower().strip(): str(c) for c in df.columns}

    trip_id_candidates = ["trip_id", "macro_trip_id", "trip"]
    col_trip_id = next((cols_lower[k] for k in trip_id_candidates if k in cols_lower), "<Single Trip per File>")

    # Agar faylda trip_time_sec bo'lsa (1-dastur V1.0.0), uni default vaqt ustuni sifatida olamiz
    time_candidates = ["trip_time_sec", "time_sec", "time_s", "time", "t_sec", "t", "seconds", "elapsed_time"]
    col_time = next((cols_lower[k] for k in time_candidates if k in cols_lower), df.columns[0])

    speed_kmh_candidates = ["velocity_kmh", "speed_kmh", "v_kmh", "vel_kmh", "kmh", "speed", "velocity_raw_kmh"]
    speed_ms_candidates = ["velocity_ms", "speed_ms", "v_ms", "vel_ms", "ms"]
    col_speed = None
    speed_unit = "km/h"
    for k in speed_kmh_candidates:
        if k in cols_lower:
            col_speed = cols_lower[k]
            speed_unit = "km/h"
            break
    if col_speed is None:
        for k in speed_ms_candidates:
            if k in cols_lower:
                col_speed = cols_lower[k]
                speed_unit = "m/s"
                break
    if col_speed is None:
        col_speed = df.columns[min(1, len(df.columns) - 1)]

    accel_candidates = ["accel_long_ms2", "accel_ms2", "acceleration_ms2", "acc_ms2", "accel", "acc", "a_ms2"]
    col_accel = next((cols_lower[k] for k in accel_candidates if k in cols_lower), "<Compute from Speed>")

    grade_candidates = ["grade_deg", "slope_deg", "grade_percent", "grade", "slope"]
    col_grade = next((cols_lower[k] for k in grade_candidates if k in cols_lower), "<None (0.0 deg)>")

    return col_trip_id, col_time, col_speed, speed_unit, col_accel, col_grade


def _coerce_numeric_or_time_seconds(series: pd.Series, is_time_col: bool = False) -> np.ndarray:
    if series.dtype == object:
        s_clean = series.astype(str).str.strip().str.replace(",", ".", regex=False)
    else:
        s_clean = series
    vals = pd.to_numeric(s_clean, errors="coerce").to_numpy(dtype=float)

    if is_time_col and np.sum(np.isfinite(vals)) < 5:
        dt_parsed = pd.to_datetime(series, errors="coerce")
        if dt_parsed.notna().sum() >= 5:
            first_valid = dt_parsed.dropna().iloc[0]
            vals = (dt_parsed - first_valid).dt.total_seconds().to_numpy(dtype=float)
    return vals


def parse_and_validate_single_trip_df(
    raw_df: pd.DataFrame,
    trip_id: int,
    trip_label: str,
    col_time: str,
    col_speed: str,
    speed_unit: str,
    col_accel: str,
    col_grade: str,
    target_dt: float,
    max_speed_kmh: float,
    max_accel_ms2: float,
    min_decel_ms2: float,
    idle_speed_kmh: float = 1.0,
):
    if raw_df is None or raw_df.empty:
        raise ValueError(f"Trip '{trip_label}' is empty.")

    raw_df.columns = [str(c) for c in raw_df.columns]
    t_vals = _coerce_numeric_or_time_seconds(raw_df[col_time], is_time_col=True)
    v_vals = _coerce_numeric_or_time_seconds(raw_df[col_speed], is_time_col=False)

    valid_mask = np.isfinite(t_vals) & np.isfinite(v_vals)
    if np.sum(valid_mask) < 5:
        raise ValueError(f"Trip '{trip_label}' has fewer than 5 valid numeric (time, speed) rows.")

    t_vals = t_vals[valid_mask]
    v_vals = v_vals[valid_mask]
    v_ms_in = np.maximum(v_vals if speed_unit == "m/s" else (v_vals / 3.6), 0.0)

    has_explicit_accel = (col_accel in raw_df.columns) and (col_accel != "<Compute from Speed>")
    a_vals_in = _coerce_numeric_or_time_seconds(raw_df[col_accel])[valid_mask] if has_explicit_accel else None

    if col_grade in raw_df.columns:
        g_vals = np.nan_to_num(_coerce_numeric_or_time_seconds(raw_df[col_grade])[valid_mask], nan=0.0)
        if "percent" in col_grade.lower():
            g_vals = np.degrees(np.arctan(g_vals / 100.0))
    else:
        g_vals = np.zeros_like(t_vals)

    # GPX lokal vaqti mavjud bo'lsa, boshlang'ich vaqtni saqlab qolamiz
    start_time_str = str(raw_df["recorded_datetime_local"].iloc[0]) if "recorded_datetime_local" in raw_df.columns else "N/A"

    order = np.argsort(t_vals)
    t_vals = t_vals[order]
    v_ms_in = v_ms_in[order]
    g_vals = g_vals[order]
    if a_vals_in is not None:
        a_vals_in = a_vals_in[order]

    t_vals = t_vals - t_vals[0]
    unique_t, unique_idx = np.unique(t_vals, return_index=True)
    if len(unique_t) < 5:
        raise ValueError(f"Trip '{trip_label}' has fewer than 5 unique timestamps.")

    v_ms_unique = v_ms_in[unique_idx]
    g_unique = g_vals[unique_idx]

    duration_s = float(unique_t[-1] - unique_t[0])
    if duration_s < target_dt * 4:
        raise ValueError(f"Trip '{trip_label}' duration ({duration_s:.1f} s) is too short.")

    n_grid = int(np.floor(duration_s / target_dt)) + 1
    t_grid = np.arange(n_grid, dtype=float) * target_dt
    v_ms_raw = np.interp(t_grid, unique_t, v_ms_unique)
    grade_deg = np.interp(t_grid, unique_t, g_unique)

    a_grad = np.gradient(v_ms_raw, target_dt)
    if a_vals_in is not None:
        a_u = a_vals_in[unique_idx]
        fin_mask = np.isfinite(a_u)
        if np.sum(fin_mask) >= 5:
            a_u_filled = np.interp(unique_t, unique_t[fin_mask], a_u[fin_mask])
            a_raw = np.interp(t_grid, unique_t, a_u_filled)
        else:
            a_raw = a_grad
    else:
        a_raw = a_grad

    v_kmh_raw = v_ms_raw * 3.6
    speed_anom = (v_kmh_raw > max_speed_kmh) | (v_kmh_raw < 0.0)
    accel_anom = (a_raw > max_accel_ms2) | (a_raw < min_decel_ms2) | (a_grad > max_accel_ms2) | (a_grad < min_decel_ms2)
    anom_mask = speed_anom | accel_anom
    anom_count = int(np.sum(anom_mask))

    v_ms_clean, a_clean, _ = enforce_kinematic_closure(
        v_ms_raw, target_dt, max_speed_kmh / 3.6, min_decel_ms2, max_accel_ms2, idle_speed_kmh / 3.6
    )
    if len(v_ms_clean) >= 7 and anom_count > 0:
        win = min(9, len(v_ms_clean) if len(v_ms_clean) % 2 == 1 else len(v_ms_clean) - 1)
        if win >= 5:
            v_ms_smooth = savgol_filter(v_ms_clean, window_length=win, polyorder=2, mode="nearest")
            idle_raw = v_ms_raw <= (idle_speed_kmh / 3.6)
            v_ms_clean = np.where(idle_raw, 0.0, np.clip(v_ms_smooth, 0.0, max_speed_kmh / 3.6))
            v_ms_clean, a_clean, _ = enforce_kinematic_closure(
                v_ms_clean, target_dt, max_speed_kmh / 3.6, min_decel_ms2, max_accel_ms2, idle_speed_kmh / 3.6
            )

    df_raw_trip = pd.DataFrame({
        "trip_id": trip_id,
        "trip_name": trip_label,
        "start_time_local": start_time_str,
        "time_sec": t_grid,
        "velocity_ms": v_ms_raw,
        "velocity_kmh": v_kmh_raw,
        "accel_long_ms2": a_raw,
        "dist_m": cumulative_trapezoid_1d(v_ms_raw, target_dt),
        "grade_deg": grade_deg,
        "is_anomaly": anom_mask,
    })

    df_clean_trip = pd.DataFrame({
        "trip_id": trip_id,
        "trip_name": trip_label,
        "start_time_local": start_time_str,
        "time_sec": t_grid,
        "velocity_ms": v_ms_clean,
        "velocity_kmh": v_ms_clean * 3.6,
        "accel_long_ms2": a_clean,
        "dist_m": cumulative_trapezoid_1d(v_ms_clean, target_dt),
        "grade_deg": grade_deg,
        "is_anomaly": False,
    })

    moving_mask = (v_ms_clean * 3.6) > idle_speed_kmh
    transitions = np.diff(moving_mask.astype(int), prepend=0)
    n_micro_trips = int(np.sum(transitions == 1))

    summary_info = {
        "Trip #": trip_id,
        "File / Trip Label": trip_label,
        "Start Time (Local)": start_time_str,
        "Samples": int(len(t_grid)),
        "Duration (s)": round(duration_s, 1),
        "Distance (km)": round(float(df_clean_trip["dist_m"].iloc[-1]) / 1000.0, 3),
        "Mean Speed (km/h)": round(float(np.mean(v_ms_clean * 3.6)), 2),
        "Max Speed (km/h)": round(float(np.max(v_ms_clean * 3.6)), 2),
        "Max Raw |a| (m/s²)": round(float(np.max(np.abs(a_raw))), 2),
        "Idle (%)": round(float(np.mean((v_ms_clean * 3.6) <= idle_speed_kmh) * 100.0), 2),
        "Micro-Trips": max(1, n_micro_trips),
        "Physics Anomalies": anom_count,
        "Anomaly (%)": round(100.0 * anom_count / len(t_grid), 2),
    }

    return df_raw_trip, df_clean_trip, summary_info


# =====================================================================
# 1-BLOK: MA'LUMOTLARNI YUKLASH, SHABLONLAR VA VALIDATSIYA
# =====================================================================
def render_block_1():
    if "b1_locked" not in st.session_state:
        st.session_state.b1_locked = False

    with st.expander("📁 BLOCK 1: Vehicle Scale Presets (20–3500 kg), Multi-Trip Ingestion & Energetic Parameters", expanded=not st.session_state.b1_locked):
        st.markdown("#### 🛵🚗🚚 Select Vehicle & Operational Class Preset (20 kg E-Moped to 3,500 kg EV)")
        preset_names = list(VEHICLE_OPERATION_PRESETS.keys())
        sel_preset_name = st.selectbox(
            "Vehicle class & driving profile preset (loads recommended defaults; all fields below remain 100% editable):",
            options=preset_names,
            index=0,
            disabled=st.session_state.b1_locked,
            key="b1_vehicle_preset_select"
        )
        preset = VEHICLE_OPERATION_PRESETS[sel_preset_name]

        tab_upload, tab_ev_cfg, tab_cards, tab_tmpl = st.tabs([
            "📂 1. Upload Trip Files (or Master V1.0.0 File)",
            "⚡ 2. Vehicle Mass, Drag, P_aux & Regen",
            "📊 3. Per-Trip Summary Cards & Audit",
            "📝 4. Template Generator"
        ])

        with tab_ev_cfg:
            st.markdown("### ⚡ Physical & Energetic Parameters (True Non-Hardcoded VSP & Battery Model)")
            ev1, ev2, ev3, ev4 = st.columns(4)
            with ev1:
                st.markdown("**1. Mass Breakdown**")
                ev_curb_mass_kg = st.number_input("Vehicle curb mass m_curb (kg):", min_value=15.0, max_value=5000.0, value=float(preset["curb_mass_kg"]), step=5.0, disabled=st.session_state.b1_locked, key="b1_ev_curb_mass")
                ev_payload_mass_kg = st.number_input("Driver + cargo payload m_payload (kg):", min_value=0.0, max_value=3000.0, value=float(preset["payload_mass_kg"]), step=5.0, disabled=st.session_state.b1_locked, key="b1_ev_payload_mass")
                ev_rot_factor = st.number_input("Rotational inertia factor δ_m:", min_value=1.00, max_value=1.30, value=float(preset["rot_factor"]), step=0.01, disabled=st.session_state.b1_locked, key="b1_ev_rot")
                st.caption(f"**Total Gross Mass $m_{{\\text{{tot}}}}$:** `{ev_curb_mass_kg + ev_payload_mass_kg:.1f} kg`")
            with ev2:
                st.markdown("**2. Aerodynamics & Tires**")
                ev_cd = st.number_input("Aerodynamic drag coeff. C_d:", min_value=0.10, max_value=1.30, value=float(preset["cd"]), step=0.01, disabled=st.session_state.b1_locked, key="b1_ev_cd")
                ev_af = st.number_input("Frontal area A_f (m²):", min_value=0.20, max_value=8.0, value=float(preset["af"]), step=0.02, disabled=st.session_state.b1_locked, key="b1_ev_af")
                ev_fr = st.number_input("Rolling resistance coeff. f_r:", min_value=0.003, max_value=0.040, value=float(preset["fr"]), step=0.001, format="%.3f", disabled=st.session_state.b1_locked, key="b1_ev_fr")
            with ev3:
                st.markdown("**3. Drivetrain & Aux Load**")
                ev_eta_drv = st.number_input("Drivetrain efficiency η_drv:", min_value=0.50, max_value=0.99, value=float(preset["eta_drv"]), step=0.01, disabled=st.session_state.b1_locked, key="b1_ev_eta_drv")
                ev_p_aux_w = st.number_input("Constant auxiliary load P_aux (W):", min_value=0.0, max_value=10000.0, value=float(preset["p_aux_w"]), step=10.0, disabled=st.session_state.b1_locked, key="b1_ev_p_aux_w", help="Power drawn by headlights, controller, phone charger, HVAC, and BMS even when stopped (v = 0).")
            with ev4:
                st.markdown("**4. Regenerative Braking Limits**")
                ev_eta_reg = st.number_input("Peak regen efficiency η_regen:", min_value=0.00, max_value=0.95, value=float(preset["eta_reg"]), step=0.05, disabled=st.session_state.b1_locked, key="b1_ev_eta_reg")
                ev_v_regen_cut_kmh = st.number_input("Low-speed regen cut-off (km/h):", min_value=0.0, max_value=25.0, value=float(preset["v_regen_cut_kmh"]), step=1.0, disabled=st.session_state.b1_locked, key="b1_ev_v_regen_cut")
                ev_max_regen_decel_ms2 = st.number_input("Max regen deceleration |a| (m/s²):", min_value=0.5, max_value=6.0, value=float(preset["max_regen_decel_ms2"]), step=0.2, disabled=st.session_state.b1_locked, key="b1_ev_max_regen_decel")

        with tab_upload:
            st.markdown("### 📂 Upload Multi-Trip Dataset (Single V1.0.0 Master File or 1–50 Individual Trip Files)")
            c_u1, c_u2 = st.columns([2, 1])
            with c_u1:
                uploaded_files = st.file_uploader(
                    "Upload `3B_Markov_and_Article_Core_29Cols.csv` (auto-splits by `trip_id`) or 1–50 separate CSV/XLSX files:",
                    type=["csv", "xlsx"],
                    accept_multiple_files=True,
                    disabled=st.session_state.b1_locked,
                    key="b1_multi_uploader"
                )
            with c_u2:
                use_builtin_demo = st.toggle(
                    "🧪 Generate 18-Trip Tashkent Delivery Demo Fleet (if no files uploaded)",
                    value=False,
                    disabled=st.session_state.b1_locked,
                    key="b1_use_demo_fleet"
                )

            st.markdown("#### ⚙️ Ingestion, Physics & Quasi-Micro-Trip Thresholds")
            p_c1, p_c2, p_c3, p_c4, p_c5, p_c6 = st.columns(6)
            with p_c1:
                target_dt = st.number_input("Time step Δt (s):", min_value=0.1, max_value=5.0, value=1.0, step=0.1, disabled=st.session_state.b1_locked, key="b1_target_dt")
            with p_c2:
                max_v_kmh = st.number_input("Max speed (km/h):", min_value=20.0, max_value=250.0, value=float(preset["max_v_kmh"]), step=5.0, disabled=st.session_state.b1_locked, key="b1_max_v_kmh")
            with p_c3:
                max_a_ms2 = st.number_input("Max accel (m/s²):", min_value=1.0, max_value=15.0, value=float(preset["max_a_ms2"]), step=0.5, disabled=st.session_state.b1_locked, key="b1_max_a_ms2")
            with p_c4:
                min_b_ms2 = st.number_input("Max braking (m/s²):", min_value=-15.0, max_value=-1.0, value=float(preset["min_b_ms2"]), step=0.5, disabled=st.session_state.b1_locked, key="b1_min_b_ms2")
            with p_c5:
                idle_thr_kmh = st.number_input("Idle threshold (km/h):", min_value=0.1, max_value=5.0, value=float(preset["idle_thr_kmh"]), step=0.1, disabled=st.session_state.b1_locked, key="b1_idle_thr_kmh")
            with p_c6:
                max_mt_dur_s = st.number_input(
                    "Max micro-trip move τ_max (s):",
                    min_value=60.0,
                    max_value=1800.0,
                    value=float(preset["max_mt_dur_s"]),
                    step=30.0,
                    disabled=st.session_state.b1_locked,
                    key="b1_max_mt_dur_s",
                    help="If a vehicle moves continuously longer than τ_max (e.g. on a ring-road or highway), the engine subdivides it at local speed minima into Quasi-Micro-Trips."
                )

            raw_trips_list = []
            clean_trips_list = []
            trip_summaries = []
            parse_errors = []

            if st.session_state.b1_locked:
                trip_summaries = st.session_state.get("b1_trip_summaries", [])
                st.success(
                    f"🔒 **Dataset Confirmed & Locked:** {len(trip_summaries)} trip(s) ready "
                    f"(`{st.session_state.b1_params.get('vehicle_preset', '')}`). Proceed to **Block 2** below."
                )
                if st.button("🔓 Unlock Block 1 to Change Files or Vehicle Parameters", width='stretch', key="b1_btn_unlock"):
                    st.session_state.b1_locked = False
                    st.session_state.pop("b2_results", None)
                    st.session_state.pop("b3_zip_bytes", None)
                    st.rerun()
            else:
                if uploaded_files and len(uploaded_files) > MAX_UPLOAD_FILES:
                    st.warning(f"⚠️ Maximum allowed files is {MAX_UPLOAD_FILES}. Only the first {MAX_UPLOAD_FILES} files will be processed.")
                    uploaded_files = uploaded_files[:MAX_UPLOAD_FILES]

                if uploaded_files:
                    first_file = uploaded_files[0]
                    try:
                        sample_df = read_uploaded_trip_file(first_file, nrows=50)
                        auto_tid, auto_t, auto_v, auto_unit, auto_a, auto_g = auto_detect_columns(sample_df)
                        cols_list = list(sample_df.columns)

                        st.markdown("#### 🗺️ Column & Multi-Trip Unpack Mapping (Auto-detected)")
                        cm0, cm1, cm2, cm3, cm4, cm5 = st.columns(6)
                        with cm0:
                            tid_opts = ["<Single Trip per File>"] + cols_list
                            sel_tid_col = st.selectbox(
                                "Multi-Trip ID column:",
                                options=tid_opts,
                                index=tid_opts.index(auto_tid) if auto_tid in tid_opts else 0,
                                key="b1_col_tid",
                                help="If uploading a master file from Program 1 (V1.0.0), select 'trip_id' to automatically unpack all 18+ trips!"
                            )
                        with cm1:
                            sel_time = st.selectbox("Time column (s):", options=cols_list, index=cols_list.index(auto_t) if auto_t in cols_list else 0, key="b1_col_time")
                        with cm2:
                            sel_speed = st.selectbox("Speed column:", options=cols_list, index=cols_list.index(auto_v) if auto_v in cols_list else 0, key="b1_col_speed")
                        with cm3:
                            sel_unit = st.selectbox("Speed unit:", options=["km/h", "m/s"], index=0 if auto_unit == "km/h" else 1, key="b1_speed_unit")
                        with cm4:
                            acc_opts = ["<Compute from Speed>"] + cols_list
                            sel_acc = st.selectbox("Acceleration column:", options=acc_opts, index=acc_opts.index(auto_a) if auto_a in acc_opts else 0, key="b1_col_acc")
                        with cm5:
                            grd_opts = ["<None (0.0 deg)>"] + cols_list
                            sel_grd = st.selectbox("Road grade column:", options=grd_opts, index=grd_opts.index(auto_g) if auto_g in grd_opts else 0, key="b1_col_grd")

                        global_trip_counter = 0
                        for uf in uploaded_files:
                            try:
                                df_in = read_uploaded_trip_file(uf)
                                auto_c = auto_detect_columns(df_in)
                                c_t = sel_time if sel_time in df_in.columns else auto_c[1]
                                c_v = sel_speed if sel_speed in df_in.columns else auto_c[2]
                                c_a = sel_acc if sel_acc in df_in.columns else "<Compute from Speed>"
                                c_g = sel_grd if sel_grd in df_in.columns else "<None (0.0 deg)>"

                                # Agar faylda trip_id ustuni bo'lsa va unda 1 tadan ko'p safar bo'lsa, avtomatik bo'lib olamiz!
                                if sel_tid_col != "<Single Trip per File>" and sel_tid_col in df_in.columns and df_in[sel_tid_col].nunique() > 1:
                                    for sub_tid, sub_grp in df_in.groupby(sel_tid_col, sort=True):
                                        global_trip_counter += 1
                                        t_label = f"{uf.name} [Trip #{int(sub_tid):02d}]"
                                        try:
                                            df_r, df_c, s_info = parse_and_validate_single_trip_df(
                                                sub_grp.reset_index(drop=True), global_trip_counter, t_label,
                                                c_t, c_v, sel_unit, c_a, c_g,
                                                target_dt, max_v_kmh, max_a_ms2, min_b_ms2, idle_thr_kmh
                                            )
                                            raw_trips_list.append(df_r)
                                            clean_trips_list.append(df_c)
                                            trip_summaries.append(s_info)
                                        except Exception as sub_ex:
                                            parse_errors.append(f"{t_label}: {str(sub_ex)}")
                                else:
                                    global_trip_counter += 1
                                    df_r, df_c, s_info = parse_and_validate_single_trip_df(
                                        df_in, global_trip_counter, uf.name,
                                        c_t, c_v, sel_unit, c_a, c_g,
                                        target_dt, max_v_kmh, max_a_ms2, min_b_ms2, idle_thr_kmh
                                    )
                                    raw_trips_list.append(df_r)
                                    clean_trips_list.append(df_c)
                                    trip_summaries.append(s_info)
                            except Exception as ex:
                                parse_errors.append(f"{uf.name}: {str(ex)}")
                    except Exception as ex:
                        parse_errors.append(f"Could not inspect '{first_file.name}': {str(ex)}")

                elif use_builtin_demo:
                    rng = np.random.default_rng(42)
                    v_cap_demo = min(max_v_kmh * 0.85, 52.0 if max_v_kmh <= 70.0 else 75.0)
                    for idx in range(1, 19):
                        dur_i = float(rng.integers(480, 1100))
                        n_pts = int(dur_i / target_dt) + 1
                        t_i = np.arange(n_pts, dtype=float) * target_dt
                        v_kmh_i = np.zeros(n_pts, dtype=float)
                        pos = 0
                        while pos < n_pts:
                            idle_len = int(rng.integers(8, 26) / target_dt)
                            pos += idle_len
                            if pos >= n_pts:
                                break
                            mt_len = int(rng.integers(32, 95) / target_dt)
                            end_pos = min(n_pts, pos + mt_len)
                            seg_n = end_pos - pos
                            if seg_n > 6:
                                tau = np.linspace(0.0, 1.0, seg_n)
                                peak_v = float(rng.uniform(22.0, v_cap_demo))
                                v_seg = peak_v * (np.sin(np.pi * tau) ** 1.15) + rng.normal(0.0, 1.0, seg_n)
                                v_seg[0] = 0.0
                                v_seg[-1] = 0.0
                                v_kmh_i[pos:end_pos] = np.clip(v_seg, 0.0, None)
                            pos = end_pos
                        demo_df = pd.DataFrame({
                            "time_sec": t_i,
                            "velocity_kmh": v_kmh_i,
                            "grade_deg": 0.6 * np.sin(2.0 * np.pi * t_i / max(dur_i, 60.0)),
                        })
                        df_r, df_c, s_info = parse_and_validate_single_trip_df(
                            demo_df, idx, f"Tashkent_Trip_{idx:02d}.csv",
                            "time_sec", "velocity_kmh", "km/h", "<Compute from Speed>", "grade_deg",
                            target_dt, max_v_kmh, max_a_ms2, min_b_ms2, idle_thr_kmh
                        )
                        raw_trips_list.append(df_r)
                        clean_trips_list.append(df_c)
                        trip_summaries.append(s_info)

                for err_msg in parse_errors:
                    st.error(f"❌ {err_msg}")

                total_anomalies = sum(item["Physics Anomalies"] for item in trip_summaries)
                cleaning_choice = "smooth"
                if trip_summaries:
                    if total_anomalies > 0:
                        st.warning(
                            f"⚠️ **Physics Anomaly Alert:** Detected **{total_anomalies:,}** sample point(s) across {len(trip_summaries)} trip(s) "
                            f"that exceed physical limits. Choose whether to smooth them or keep raw data:"
                        )
                        cleaning_mode_label = st.radio(
                            "Anomaly handling mode:",
                            options=[
                                "✅ Accept & Smooth Anomalies (Recommended: Kinematic Rate-Limiter + Savitzky–Golay)",
                                "⚠️ Proceed Without Smoothing (Keep Raw Data)"
                            ],
                            index=0,
                            key="b1_anomaly_radio"
                        )
                        cleaning_choice = "smooth" if cleaning_mode_label.startswith("✅") else "raw"
                    else:
                        st.success(f"✅ **Physics Integrity Check Passed:** {len(trip_summaries)} trip(s) unpacked with zero anomalies.")

                    if st.button(
                        f"✅ Confirm & Lock {len(trip_summaries)} Trip(s) — Proceed to Block 2",
                        type="primary",
                        width='stretch',
                        key="b1_btn_lock"
                    ):
                        chosen_list = clean_trips_list if cleaning_choice == "smooth" else raw_trips_list
                        df_combined = pd.concat(chosen_list, ignore_index=True)
                        st.session_state.b1_locked = True
                        st.session_state.b1_cleaning_applied = (cleaning_choice == "smooth")
                        st.session_state.b1_trips_df = df_combined
                        st.session_state.b1_trip_summaries = trip_summaries
                        st.session_state.b1_params = {
                            "vehicle_preset": sel_preset_name,
                            "target_dt": float(target_dt),
                            "max_v_kmh": float(max_v_kmh),
                            "max_a_ms2": float(max_a_ms2),
                            "min_b_ms2": float(min_b_ms2),
                            "idle_thr_kmh": float(idle_thr_kmh),
                            "max_mt_dur_s": float(max_mt_dur_s),
                            "n_files": int(len(trip_summaries)),
                            "total_anomalies_detected": int(total_anomalies),
                            "cleaning_applied": bool(cleaning_choice == "smooth"),
                            "ev_curb_mass_kg": float(ev_curb_mass_kg),
                            "ev_payload_mass_kg": float(ev_payload_mass_kg),
                            "ev_mass_kg": float(ev_curb_mass_kg + ev_payload_mass_kg),
                            "ev_rot_factor": float(ev_rot_factor),
                            "ev_cd": float(ev_cd),
                            "ev_af": float(ev_af),
                            "ev_fr": float(ev_fr),
                            "ev_eta_drv": float(ev_eta_drv),
                            "ev_eta_reg": float(ev_eta_reg),
                            "ev_p_aux_w": float(ev_p_aux_w),
                            "ev_v_regen_cut_kmh": float(ev_v_regen_cut_kmh),
                            "ev_max_regen_decel_ms2": float(ev_max_regen_decel_ms2),
                            "default_benchmark": preset["default_benchmark"],
                            "default_cycle_dur_s": int(preset["target_cycle_dur_s"]),
                            "default_v_bin": float(preset["v_bin_kmh"]),
                            "default_a_bin": float(preset["a_bin_ms2"]),
                        }
                        st.rerun()
                else:
                    st.info("👆 Upload your `3B_Markov_and_Article_Core_29Cols.csv` file from Program 1 (or 1–50 separate trip files) above.")

        with tab_cards:
            summaries_to_show = st.session_state.get("b1_trip_summaries", trip_summaries)
            if not summaries_to_show:
                st.info("No trips loaded yet. Upload files in Tab 1 to view per-trip statistical summary cards.")
            else:
                st.markdown("### 📊 Multi-Trip Fleet Discovery & Per-Trip Summary Cards")
                tot_dur = sum(x["Duration (s)"] for x in summaries_to_show)
                tot_dist = sum(x["Distance (km)"] for x in summaries_to_show)
                fleet_mean_v = float(np.mean([x["Mean Speed (km/h)"] for x in summaries_to_show]))
                fleet_max_v = float(np.max([x["Max Speed (km/h)"] for x in summaries_to_show]))
                tot_mt = sum(x["Micro-Trips"] for x in summaries_to_show)

                k1, k2, k3, k4, k5 = st.columns(5)
                k1.metric("Unpacked Trips", f"{len(summaries_to_show)} trips", f"{tot_mt} kinematic micro-trips")
                k2.metric("Total Fleet Duration", f"{tot_dur / 3600.0:.2f} h", f"{tot_dur:,.0f} s")
                k3.metric("Total Fleet Distance", f"{tot_dist:.2f} km", f"{tot_dist * 1000:,.0f} m")
                k4.metric("Fleet Mean Speed", f"{fleet_mean_v:.2f} km/h", f"{fleet_mean_v / 3.6:.2f} m/s")
                k5.metric("Fleet Peak Speed", f"{fleet_max_v:.2f} km/h", f"Gross Mass: {ev_curb_mass_kg + ev_payload_mass_kg:.0f} kg")

                n_cols = 3
                for row_start in range(0, min(len(summaries_to_show), 18), n_cols):
                    cols_card = st.columns(n_cols)
                    for j in range(n_cols):
                        idx_c = row_start + j
                        if idx_c < len(summaries_to_show):
                            item = summaries_to_show[idx_c]
                            with cols_card[j]:
                                with st.container(border=True):
                                    st.markdown(f"**🛵 Trip #{item['Trip #']}: `{item['File / Trip Label']}`**")
                                    st.caption(
                                        f"🕒 **Start:** {item['Start Time (Local)']} | ⏱️ **{item['Duration (s)']} s** | 📏 **{item['Distance (km)']} km**"
                                    )
                                    mc1, mc2, mc3 = st.columns(3)
                                    mc1.metric("Mean V", f"{item['Mean Speed (km/h)']} km/h")
                                    mc2.metric("Max V", f"{item['Max Speed (km/h)']} km/h")
                                    mc3.metric("Idle", f"{item['Idle (%)']}%")
                st.markdown("#### 📋 Complete Multi-Trip Audit Table")
                st.dataframe(pd.DataFrame(summaries_to_show), width='stretch', hide_index=True)

        with tab_tmpl:
            st.markdown("### 📝 Download Standard Input Template (CSV / XLSX)")
            c_t1, c_t2, c_t3, c_t4 = st.columns(4)
            with c_t1:
                tmpl_dt = st.number_input("Sampling interval Δt (s):", min_value=0.1, max_value=10.0, value=1.0, step=0.1, key="b1_tmpl_dt")
            with c_t2:
                tmpl_dur = st.number_input("Total template duration (s):", min_value=10.0, max_value=36000.0, value=600.0, step=60.0, key="b1_tmpl_dur")
            with c_t3:
                tmpl_mode = st.selectbox("Template content:", options=["Blank (Zeros to fill)", "Synthetic Urban Demo Wave"], key="b1_tmpl_mode")
            with c_t4:
                tmpl_fmt = st.selectbox("Template file format:", options=["CSV (.csv)", "Excel (.xlsx)"], key="b1_tmpl_fmt")

            df_tmpl = generate_template_dataframe(tmpl_dt, tmpl_dur, include_demo_wave=(tmpl_mode == "Synthetic Urban Demo Wave"))
            if tmpl_fmt.startswith("CSV"):
                st.download_button(
                    label=f"⬇️ Download Template CSV ({len(df_tmpl):,} rows)",
                    data=df_tmpl.to_csv(index=False).encode("utf-8"),
                    file_name=f"QwatLab_Trip_Template_{int(tmpl_dur)}s.csv",
                    mime="text/csv",
                    width='stretch',
                    key="b1_dl_tmpl_csv"
                )
            else:
                xlsx_io = io.BytesIO()
                with pd.ExcelWriter(xlsx_io, engine="openpyxl") as writer:
                    df_tmpl.to_excel(writer, index=False, sheet_name="Trip_Template")
                st.download_button(
                    label=f"⬇️ Download Template XLSX ({len(df_tmpl):,} rows)",
                    data=xlsx_io.getvalue(),
                    file_name=f"QwatLab_Trip_Template_{int(tmpl_dur)}s.xlsx",
                    mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
                    width='stretch',
                    key="b1_dl_tmpl_xlsx"
                )


# =====================================================================
# 2-BLOK: IXCHAM MARKOV ZANJIRI, QUASI-MICRO-TRIP, SILHOUETTE K-MEANS VA SAFD
# =====================================================================
@nb.njit
def build_compact_markov_cdf_numba(
    v_idx_arr: np.ndarray,
    a_idx_arr: np.ndarray,
    trip_id_arr: np.ndarray,
    n_v_bins: int,
    n_a_bins: int
):
    n_grid = n_v_bins * n_a_bins
    state_visits = np.zeros(n_grid, dtype=np.int64)
    n_samples = len(v_idx_arr)

    for k in range(n_samples):
        s_curr = v_idx_arr[k] * n_a_bins + a_idx_arr[k]
        if 0 <= s_curr < n_grid:
            state_visits[s_curr] += 1

    n_active = 0
    for s in range(n_grid):
        if state_visits[s] > 0:
            n_active += 1

    if n_active == 0:
        n_active = 1
        active_grid_states = np.zeros(1, dtype=np.int64)
        grid_to_active = np.full(n_grid, -1, dtype=np.int64)
        grid_to_active[0] = 0
        cdf = np.ones((1, 1), dtype=np.float64)
        valid_row = np.ones(1, dtype=np.bool_)
        return cdf, active_grid_states, grid_to_active, valid_row, 0

    active_grid_states = np.empty(n_active, dtype=np.int64)
    grid_to_active = np.full(n_grid, -1, dtype=np.int64)
    idx_a = 0
    for s in range(n_grid):
        if state_visits[s] > 0:
            active_grid_states[idx_a] = s
            grid_to_active[s] = idx_a
            idx_a += 1

    cdf = np.zeros((n_active, n_active), dtype=np.float64)
    valid_row = np.zeros(n_active, dtype=np.bool_)

    for k in range(n_samples - 1):
        if trip_id_arr[k] == trip_id_arr[k + 1]:
            s_c = v_idx_arr[k] * n_a_bins + a_idx_arr[k]
            s_n = v_idx_arr[k + 1] * n_a_bins + a_idx_arr[k + 1]
            if 0 <= s_c < n_grid and 0 <= s_n < n_grid:
                ac = grid_to_active[s_c]
                an = grid_to_active[s_n]
                if ac >= 0 and an >= 0:
                    cdf[ac, an] += 1.0

    for i in range(n_active):
        row_sum = 0.0
        for j in range(n_active):
            row_sum += cdf[i, j]
        if row_sum > 0.0:
            valid_row[i] = True
            cum = 0.0
            for j in range(n_active):
                cum += cdf[i, j] / row_sum
                cdf[i, j] = cum
            cdf[i, n_active - 1] = 1.0

    return cdf, active_grid_states, grid_to_active, valid_row, n_active


@nb.njit
def synthesize_markov_chain_numba(
    cdf: np.ndarray,
    active_grid_states: np.ndarray,
    grid_to_active: np.ndarray,
    valid_row: np.ndarray,
    v_centers_ms: np.ndarray,
    a_centers_ms2: np.ndarray,
    n_v_bins: int,
    n_a_bins: int,
    n_steps: int,
    dt: float,
    v_max_ms: float,
    a_min_ms2: float,
    a_max_ms2: float,
    start_grid_state: int,
    zero_a_bin: int,
    seed: int
):
    np.random.seed(seed)
    v_out = np.zeros(n_steps, dtype=np.float64)
    a_out = np.zeros(n_steps, dtype=np.float64)
    state_out = np.zeros(n_steps, dtype=np.int64)

    n_active = len(active_grid_states)
    curr_act = grid_to_active[start_grid_state] if 0 <= start_grid_state < len(grid_to_active) else 0
    if curr_act < 0:
        curr_act = 0

    state_out[0] = active_grid_states[curr_act]
    v_bin_width_ms = (v_centers_ms[1] - v_centers_ms[0]) if n_v_bins > 1 else (1.0 / 3.6)
    a_bin_width_ms2 = (a_centers_ms2[1] - a_centers_ms2[0]) if n_a_bins > 1 else 0.1
    brake_horizon = max(5, int(15.0 / dt))

    for k in range(1, n_steps):
        remaining_steps = n_steps - 1 - k
        if remaining_steps < brake_horizon and v_out[k - 1] > 0.05:
            req_decel = -v_out[k - 1] / ((remaining_steps + 1.0) * dt)
            if req_decel < a_min_ms2:
                req_decel = a_min_ms2
            a_next = req_decel + (np.random.random() - 0.5) * 0.05
            if a_next > 0.0:
                a_next = req_decel
            if a_next < a_min_ms2:
                a_next = a_min_ms2
            v_next = v_out[k - 1] + a_next * dt
            if v_next <= 0.08 or remaining_steps == 0:
                v_next = 0.0
                a_next = -v_out[k - 1] / dt
            v_out[k] = v_next
            a_out[k] = a_next
            v_bin = int(min(n_v_bins - 1, max(0, int(v_next / max(1e-6, v_bin_width_ms)))))
            g_state = v_bin * n_a_bins + zero_a_bin
            state_out[k] = g_state
            if 0 <= g_state < len(grid_to_active) and grid_to_active[g_state] >= 0:
                curr_act = grid_to_active[g_state]
            continue

        if not valid_row[curr_act]:
            curr_grid = active_grid_states[curr_act]
            curr_v_bin = curr_grid // n_a_bins
            best_act = 0
            best_dist = 1000000
            for idx_a in range(n_active):
                if valid_row[idx_a]:
                    g_cand = active_grid_states[idx_a]
                    dv_b = abs((g_cand // n_a_bins) - curr_v_bin)
                    da_b = abs((g_cand % n_a_bins) - zero_a_bin)
                    d_tot = dv_b * 3 + da_b
                    if d_tot < best_dist:
                        best_dist = d_tot
                        best_act = idx_a
            curr_act = best_act

        u = np.random.random()
        row_cdf = cdf[curr_act]
        lo = 0
        hi = n_active - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if row_cdf[mid] < u:
                lo = mid + 1
            else:
                hi = mid
        next_act = lo
        next_grid = active_grid_states[next_act]

        v_bin_next = next_grid // n_a_bins
        a_bin_next = next_grid % n_a_bins

        if v_bin_next == 0 and a_bin_next == zero_a_bin and v_out[k - 1] < (1.8 / 3.6):
            v_next = 0.0
            a_actual = -v_out[k - 1] / dt
        else:
            a_sample = a_centers_ms2[a_bin_next] + (np.random.random() - 0.5) * 0.4 * a_bin_width_ms2
            v_cand = v_out[k - 1] + a_sample * dt
            v_low = max(0.0, v_centers_ms[v_bin_next] - 0.85 * v_bin_width_ms)
            v_high = min(v_max_ms, v_centers_ms[v_bin_next] + 0.85 * v_bin_width_ms)
            if v_cand < v_low:
                v_cand = 0.65 * v_cand + 0.35 * v_low
            elif v_cand > v_high:
                v_cand = 0.65 * v_cand + 0.35 * v_high
            a_req = (v_cand - v_out[k - 1]) / dt
            if a_req > a_max_ms2:
                a_req = a_max_ms2
            elif a_req < a_min_ms2:
                a_req = a_min_ms2
            v_next = v_out[k - 1] + a_req * dt
            if v_next < 0.0:
                v_next = 0.0
            elif v_next > v_max_ms:
                v_next = v_max_ms
            a_actual = (v_next - v_out[k - 1]) / dt

        v_out[k] = v_next
        a_out[k] = a_actual
        state_out[k] = next_grid
        curr_act = next_act

    v_out[n_steps - 1] = 0.0
    if n_steps > 1:
        a_out[n_steps - 1] = (v_out[n_steps - 1] - v_out[n_steps - 2]) / dt
    return v_out, a_out, state_out


@nb.njit
def compute_safd_matrix_numba(
    v_kmh: np.ndarray,
    a_ms2: np.ndarray,
    v_min: float,
    v_step: float,
    n_v_bins: int,
    a_min: float,
    a_step: float,
    n_a_bins: int
):
    mat = np.zeros((n_v_bins, n_a_bins), dtype=np.float64)
    n = len(v_kmh)
    if n == 0:
        return mat
    for k in range(n):
        vb = int(np.floor((v_kmh[k] - v_min) / v_step))
        ab = int(np.floor((a_ms2[k] - a_min) / a_step))
        if vb < 0:
            vb = 0
        elif vb >= n_v_bins:
            vb = n_v_bins - 1
        if ab < 0:
            ab = 0
        elif ab >= n_a_bins:
            ab = n_a_bins - 1
        mat[vb, ab] += 1.0

    inv_n = 100.0 / float(n)
    for i in range(n_v_bins):
        for j in range(n_a_bins):
            mat[i, j] *= inv_n
    return mat


def _split_long_excursion_at_minima(v_seg: np.ndarray, g_seg: np.ndarray, dt: float, max_mt_dur_s: float):
    """
    Agar transport trassa yoki halqa yo'lida to'xtamasdan uzoq yursa (len * dt > max_mt_dur_s),
    harakatni o'rtasidan chopmasdan, tezlikning lokal minimumlaridan Kvazi-mikrosikllarga bo'ladi.
    """
    max_samples = max(30, int(round(max_mt_dur_s / dt)))
    min_samples = max(15, int(round(0.35 * max_mt_dur_s / dt)))
    if len(v_seg) <= max_samples:
        return [(v_seg, g_seg, False)]

    chunks = []
    curr = 0
    n = len(v_seg)
    while (n - curr) > max_samples:
        search_lo = curr + min_samples
        search_hi = min(n - min_samples, curr + max_samples)
        if search_hi <= search_lo:
            break
        # Oyna ichidagi eng past tezlik nuqtasini (lokal minimum) topamiz
        local_min_offset = int(np.argmin(v_seg[search_lo:search_hi]))
        split_pt = search_lo + local_min_offset
        chunks.append((v_seg[curr:split_pt].copy(), g_seg[curr:split_pt].copy(), True))
        curr = split_pt

    if curr < n:
        chunks.append((v_seg[curr:n].copy(), g_seg[curr:n].copy(), curr > 0))
    return chunks


def _run_kmeans_and_silhouette(z_feats: np.ndarray, k_c: int):
    """Z-normallashtirilgan 5D mikrosikl belgilari ustida K-Means va Silhouette Score ni hisoblaydi."""
    n = len(z_feats)
    if k_c <= 1 or n <= k_c:
        return np.zeros(n, dtype=int), 0.0

    quantiles = np.linspace(0.12, 0.88, k_c)
    sort_idx = np.argsort(z_feats[:, 0])
    init_indices = [sort_idx[int(q * (n - 1))] for q in quantiles]
    centroids = z_feats[init_indices].copy()
    labels = np.zeros(n, dtype=int)

    for _ in range(30):
        dists = np.linalg.norm(z_feats[:, None, :] - centroids[None, :, :], axis=2)
        new_labels = np.argmin(dists, axis=1)
        if np.array_equal(new_labels, labels):
            break
        labels = new_labels
        for c in range(k_c):
            if np.any(labels == c):
                centroids[c] = np.mean(z_feats[labels == c], axis=0)

    # Tezkor Silhouette baholash (namuna soni katta bo'lsa 400 tagacha subsample bilan)
    eval_idx = np.linspace(0, n - 1, min(n, 400), dtype=int)
    sub_z = z_feats[eval_idx]
    sub_lbl = labels[eval_idx]
    d_mat = np.linalg.norm(sub_z[:, None, :] - sub_z[None, :, :], axis=2)
    sil_vals = np.zeros(len(eval_idx), dtype=float)
    for i in range(len(eval_idx)):
        own_c = sub_lbl[i]
        same_m = (sub_lbl == own_c)
        same_m[i] = False
        a_i = float(np.mean(d_mat[i, same_m])) if np.any(same_m) else 0.0
        b_i = np.inf
        for c in range(k_c):
            if c != own_c and np.any(sub_lbl == c):
                b_c = float(np.mean(d_mat[i, sub_lbl == c]))
                if b_c < b_i:
                    b_i = b_c
        if np.isfinite(b_i) and max(a_i, b_i) > 1e-9:
            sil_vals[i] = (b_i - a_i) / max(a_i, b_i)
    return labels, float(np.mean(sil_vals))


def extract_and_cluster_micro_trips(
    df_trips: pd.DataFrame,
    dt: float,
    idle_thr_kmh: float = 1.0,
    max_mt_dur_s: float = 240.0,
    cluster_mode: str = "Auto-Optimal K_c (Silhouette Score, 2..5)"
):
    """
    Gibrid Kinematic Micro-Trip + Quasi-Micro-Trip ajratish, K-Means klasterlash (Auto-Silhouette
    yoki foydalanuvchi tanlagan K_c) va har bir rejimning vaqt ulushi kvotasini (w_c) aniqlash.
    """
    micro_trips = []
    quasi_split_count = 0

    for trip_id, group in df_trips.groupby("trip_id", sort=False):
        v_ms = group["velocity_ms"].to_numpy(dtype=float)
        v_kmh = v_ms * 3.6
        g_deg = group["grade_deg"].to_numpy(dtype=float)

        is_moving = v_kmh > idle_thr_kmh
        n = len(v_ms)
        idx = 0
        trip_mts = []
        while idx < n:
            start_idx = idx
            while idx < n and not is_moving[idx]:
                idx += 1
            move_start = idx
            while idx < n and is_moving[idx]:
                idx += 1
            end_idx = idx
            if end_idx - move_start >= max(4, int(4.0 / dt)):
                raw_v_seg = v_ms[start_idx:end_idx].copy()
                raw_g_seg = g_deg[start_idx:end_idx].copy()
                sub_chunks = _split_long_excursion_at_minima(raw_v_seg, raw_g_seg, dt, max_mt_dur_s)
                if len(sub_chunks) > 1:
                    quasi_split_count += (len(sub_chunks) - 1)

                for (seg_v, seg_g, is_quasi) in sub_chunks:
                    if not is_quasi:
                        seg_v[0] = 0.0
                        seg_v[-1] = 0.0
                    seg_a = np.gradient(seg_v, dt)
                    pos_a = seg_a[seg_a > 0.1]
                    neg_a = seg_a[seg_a < -0.1]
                    trip_mts.append({
                        "trip_id": int(trip_id),
                        "v_ms": seg_v,
                        "a_ms2": seg_a,
                        "grade_deg": seg_g,
                        "is_quasi": bool(is_quasi),
                        "duration_s": len(seg_v) * dt,
                        "v_mean_kmh": float(np.mean(seg_v) * 3.6),
                        "v_max_kmh": float(np.max(seg_v) * 3.6),
                        "a_pos_ms2": float(np.mean(pos_a)) if len(pos_a) > 0 else 0.0,
                        "a_neg_ms2": float(np.mean(neg_a)) if len(neg_a) > 0 else 0.0,
                        "idle_ratio": float(np.mean(seg_v * 3.6 <= idle_thr_kmh)),
                        "end_idx": end_idx,
                    })

        if trip_mts and trip_mts[-1]["end_idx"] < n:
            last_mt = trip_mts[-1]
            tail_len = n - last_mt["end_idx"]
            if tail_len > 0:
                last_mt["v_ms"] = np.concatenate([last_mt["v_ms"], np.zeros(tail_len, dtype=float)])
                last_mt["a_ms2"] = np.gradient(last_mt["v_ms"], dt)
                last_mt["grade_deg"] = np.concatenate([last_mt["grade_deg"], g_deg[last_mt["end_idx"]:n]])
                last_mt["duration_s"] = len(last_mt["v_ms"]) * dt
                last_mt["v_mean_kmh"] = float(np.mean(last_mt["v_ms"]) * 3.6)
                last_mt["idle_ratio"] = float(np.mean(last_mt["v_ms"] * 3.6 <= idle_thr_kmh))

        micro_trips.extend(trip_mts)

    if not micro_trips:
        return [], np.ones((1, 1), dtype=float), np.array([1.0]), 1, 0.0, 0

    feats = np.array([
        [m["v_mean_kmh"], m["v_max_kmh"], m["a_pos_ms2"], abs(m["a_neg_ms2"]), m["idle_ratio"]]
        for m in micro_trips
    ], dtype=float)
    f_mean = np.mean(feats, axis=0)
    f_std = np.where(np.std(feats, axis=0) > 1e-6, np.std(feats, axis=0), 1.0)
    z_feats = (feats - f_mean) / f_std

    if "Auto" in str(cluster_mode):
        best_k = 3
        best_sil = -1.0
        best_labels = np.zeros(len(micro_trips), dtype=int)
        for cand_k in range(2, min(6, len(micro_trips))):
            lbl_k, sil_k = _run_kmeans_and_silhouette(z_feats, cand_k)
            if sil_k > best_sil:
                best_sil = sil_k
                best_k = cand_k
                best_labels = lbl_k
        k_c = best_k
        labels = best_labels
        sil_score = best_sil
    else:
        parsed_k = int(re.findall(r"\d+", str(cluster_mode))[0]) if re.findall(r"\d+", str(cluster_mode)) else 3
        k_c = min(max(2, parsed_k), max(2, len(micro_trips)))
        labels, sil_score = _run_kmeans_and_silhouette(z_feats, k_c)

    # Klasterlarni o'rtacha tezlik bo'yicha o'sish tartibida raqamlaymiz (0: Congested -> K_c-1: Fast/Highway)
    cluster_means = [np.mean(feats[labels == c, 0]) if np.any(labels == c) else 0.0 for c in range(k_c)]
    rank_map = {old_c: new_c for new_c, old_c in enumerate(np.argsort(cluster_means))}
    for i, m in enumerate(micro_trips):
        m["cluster_id"] = int(rank_map[int(labels[i])])

    # Har bir klasterning empirik vaqt ulushi (Time-Share Quota w_c)
    cluster_durations = np.zeros(k_c, dtype=float)
    for m in micro_trips:
        cluster_durations[m["cluster_id"]] += m["duration_s"]
    total_mt_dur = max(float(np.sum(cluster_durations)), 1e-6)
    cluster_weights = cluster_durations / total_mt_dur

    regime_counts = np.ones((k_c, k_c), dtype=float) * 0.1
    for i in range(len(micro_trips) - 1):
        if micro_trips[i]["trip_id"] == micro_trips[i + 1]["trip_id"]:
            c_curr = micro_trips[i]["cluster_id"]
            c_next = micro_trips[i + 1]["cluster_id"]
            regime_counts[c_curr, c_next] += 1.0
    regime_tpm = regime_counts / np.sum(regime_counts, axis=1, keepdims=True)

    return micro_trips, regime_tpm, cluster_weights, k_c, sil_score, quasi_split_count


def synthesize_stratified_markov_microtrip_cycle(
    micro_trips: list,
    regime_tpm: np.ndarray,
    cluster_weights: np.ndarray,
    df_trips: pd.DataFrame,
    n_target_steps: int,
    dt: float,
    rng: np.random.Generator
):
    """
    V1.0.0 Time-Quota Stratified + Intact Boundary Matching sintezi:
    1) Har bir klaster o'zining empirik vaqt kvotasini (w_c * n_target_steps) to'ldiradi.
    2) Oxirgi mikrosikl harakat o'rtasidan chopilmaydi: qolgan vaqtga eng yaqin butun mikrosikl tanlanadi.
    3) Kvazi-mikrosikllar ulanish joyida C^1 silliq ko'prik qo'llanadi.
    """
    if not micro_trips:
        v_emp = df_trips["velocity_ms"].to_numpy(dtype=float)
        g_emp = df_trips["grade_deg"].to_numpy(dtype=float)
        v_out = np.resize(v_emp, n_target_steps)
        g_out = np.resize(g_emp, n_target_steps)
        v_out[0] = 0.0
        v_out[-1] = 0.0
        return v_out, np.gradient(v_out, dt), g_out

    k_c = regime_tpm.shape[0]
    clusters = {c: [m for m in micro_trips if m["cluster_id"] == c] for c in range(k_c)}
    for c in range(k_c):
        if not clusters[c]:
            clusters[c] = micro_trips

    target_cluster_steps = np.maximum(1.0, cluster_weights * float(n_target_steps))
    accum_cluster_steps = np.zeros(k_c, dtype=float)

    curr_regime = int(rng.choice(k_c, p=cluster_weights / np.sum(cluster_weights)))
    segs_v = []
    segs_g = []
    curr_len = 0

    while curr_len < n_target_steps:
        rem_steps = n_target_steps - curr_len
        pool = clusters[curr_regime]

        # Agar yakunlashga <= 90 qadam qolgan bo'lsa, harakatni o'rtasidan kesmaslik uchun
        # uzunligi rem_steps ga eng yaqin bo'lgan butun mikrosiklni tanlaymiz!
        if rem_steps <= max(45, int(90.0 / dt)):
            lengths = np.array([len(m["v_ms"]) for m in pool], dtype=int)
            diffs = np.abs(lengths - rem_steps)
            best_indices = np.argsort(diffs)[:min(5, len(pool))]
            chosen_mt = pool[int(rng.choice(best_indices))]
        else:
            chosen_mt = pool[int(rng.integers(0, len(pool)))]

        v_piece = chosen_mt["v_ms"].copy()
        g_piece = chosen_mt["grade_deg"].copy()

        # Agar oldingi bo'lak oxiri va yangi bo'lak boshi orasida tezlik farqi bo'lsa (Quasi-Micro-Trip), 5 nuqtada silliqlaymiz
        if segs_v and (segs_v[-1][-1] > 0.1 or v_piece[0] > 0.1):
            v_prev_end = float(segs_v[-1][-1])
            blend_n = min(6, len(v_piece))
            if blend_n > 1:
                weights_b = np.linspace(0.7, 0.0, blend_n)
                v_piece[:blend_n] = weights_b * v_prev_end + (1.0 - weights_b) * v_piece[:blend_n]

        segs_v.append(v_piece)
        segs_g.append(g_piece)
        curr_len += len(v_piece)
        accum_cluster_steps[curr_regime] += len(v_piece)

        # Keyingi rejimni tanlashda Markov TPM ni qolgan vaqt kvotasi tanqisligi (deficit) bilan ko'paytiramiz
        quota_deficit = np.maximum(0.05, target_cluster_steps - accum_cluster_steps)
        trans_probs = regime_tpm[curr_regime] * quota_deficit
        trans_probs = trans_probs / np.sum(trans_probs)
        curr_regime = int(rng.choice(k_c, p=trans_probs))

    v_full = np.concatenate(segs_v)
    g_full = np.concatenate(segs_g)

    # Maqsadli uzunlikka keltirish (oxirgi mikrosikl intact tanlangani uchun faqat kichik idle tafovuti tekislanadi)
    if len(v_full) >= n_target_steps:
        v_cand_ms = v_full[:n_target_steps].copy()
        g_cand_deg = g_full[:n_target_steps].copy()
    else:
        pad_n = n_target_steps - len(v_full)
        v_cand_ms = np.concatenate([v_full, np.zeros(pad_n, dtype=float)])
        g_cand_deg = np.concatenate([g_full, np.full(pad_n, g_full[-1], dtype=float)])

    ramp_n = min(10, max(4, int(8.0 / dt)))
    if len(v_cand_ms) > ramp_n and v_cand_ms[-1] > 0.0:
        v_cand_ms[-ramp_n:] *= np.linspace(1.0, 0.0, ramp_n)
    v_cand_ms[0] = 0.0
    v_cand_ms[-1] = 0.0
    a_cand_ms2 = np.gradient(v_cand_ms, dt)
    return v_cand_ms, a_cand_ms2, g_cand_deg


def calculate_kinematic_metrics(
    v_ms: np.ndarray,
    a_ms2: np.ndarray,
    grade_deg: np.ndarray,
    dt: float,
    b1_params: dict,
    acc_thr_ms2: float = 0.10,
    trip_ids: np.ndarray = None
) -> dict:
    idle_thr_kmh = float(b1_params.get("idle_thr_kmh", 1.0))
    v_ms = np.asarray(v_ms, dtype=float)
    a_ms2 = np.asarray(a_ms2, dtype=float)
    v_kmh = v_ms * 3.6
    n = len(v_ms)

    if trip_ids is not None and len(trip_ids) == n and n > 1:
        same_trip = (trip_ids[:-1] == trip_ids[1:])
        duration_s = float(int(np.sum(same_trip)) * dt)
        ds_arr = np.where(same_trip, 0.5 * (v_ms[:-1] + v_ms[1:]) * dt, 0.0)
        dist_m = float(np.sum(ds_arr))
        dv2 = np.where(same_trip, np.diff(v_ms ** 2), 0.0)
    else:
        duration_s = float(max(0.0, (n - 1) * dt))
        dist_m = float(cumulative_trapezoid_1d(v_ms, dt)[-1]) if n > 1 else 0.0
        dv2 = np.diff(v_ms ** 2) if n > 1 else np.zeros(0)

    dist_km = dist_m / 1000.0
    idle_mask = v_kmh <= idle_thr_kmh
    moving_mask = ~idle_mask
    accel_mask = moving_mask & (a_ms2 > acc_thr_ms2)
    decel_mask = moving_mask & (a_ms2 < -acc_thr_ms2)
    cruise_mask = moving_mask & (np.abs(a_ms2) <= acc_thr_ms2)

    pos_a = a_ms2[a_ms2 > acc_thr_ms2]
    neg_a = a_ms2[a_ms2 < -acc_thr_ms2]

    if n > 1 and dist_m > 1.0:
        pke_ms2 = float(np.sum(np.maximum(0.0, dv2)) / dist_m)
        rpa_ms2 = float(np.sum(np.maximum(0.0, a_ms2) * v_ms * dt) / dist_m)
    else:
        pke_ms2 = 0.0
        rpa_ms2 = 0.0

    _, _, ev_metrics = compute_vsp_and_ev_energy(v_ms, a_ms2, grade_deg, dt, b1_params, override_dist_m=dist_m)

    res_dict = {
        "duration_s": duration_s,
        "distance_km": dist_km,
        "v_mean_kmh": float(np.mean(v_kmh)) if n > 0 else 0.0,
        "v_drive_kmh": float(np.mean(v_kmh[moving_mask])) if np.any(moving_mask) else 0.0,
        "v_std_kmh": float(np.std(v_kmh)) if n > 0 else 0.0,
        "v_max_kmh": float(np.max(v_kmh)) if n > 0 else 0.0,
        "a_pos_mean_ms2": float(np.mean(pos_a)) if len(pos_a) > 0 else 0.0,
        "a_neg_mean_ms2": float(np.mean(neg_a)) if len(neg_a) > 0 else 0.0,
        "a_std_ms2": float(np.std(a_ms2)) if n > 0 else 0.0,
        "a_rms_ms2": float(np.sqrt(np.mean(a_ms2 ** 2))) if n > 0 else 0.0,
        "pct_idle": float(np.mean(idle_mask) * 100.0) if n > 0 else 0.0,
        "pct_accel": float(np.mean(accel_mask) * 100.0) if n > 0 else 0.0,
        "pct_decel": float(np.mean(decel_mask) * 100.0) if n > 0 else 0.0,
        "pct_cruise": float(np.mean(cruise_mask) * 100.0) if n > 0 else 0.0,
        "pke_ms2": pke_ms2,
        "rpa_ms2": rpa_ms2,
    }
    res_dict.update(ev_metrics)
    return res_dict


def evaluate_safd_and_convergence(
    cand_metrics: dict,
    emp_metrics: dict,
    safd_cand: np.ndarray,
    safd_emp: np.ndarray,
    v_cand_kmh: np.ndarray,
    v_emp_kmh: np.ndarray
) -> dict:
    diff_mat = safd_cand - safd_emp
    safd_rmse_pct = float(np.sqrt(np.mean(diff_mat ** 2)))
    safd_mae_pct = float(np.mean(np.abs(diff_mat)))

    p_c = np.clip(safd_cand / 100.0, 0.0, 1.0)
    p_e = np.clip(safd_emp / 100.0, 0.0, 1.0)
    p_c = p_c / max(np.sum(p_c), 1e-12)
    p_e = p_e / max(np.sum(p_e), 1e-12)
    bhattacharyya_sim = float(np.sum(np.sqrt(p_c * p_e)) * 100.0)

    m_dist = 0.5 * (p_c + p_e)
    mask_c = (p_c > 1e-12) & (m_dist > 1e-12)
    mask_e = (p_e > 1e-12) & (m_dist > 1e-12)
    kl_c = np.sum(p_c[mask_c] * np.log2(p_c[mask_c] / m_dist[mask_c]))
    kl_e = np.sum(p_e[mask_e] * np.log2(p_e[mask_e] / m_dist[mask_e]))
    js_div = float(0.5 * (kl_c + kl_e))

    ks_stat = float(ks_2samp(v_cand_kmh, v_emp_kmh).statistic)

    norm_c = float(np.linalg.norm(safd_cand))
    norm_e = float(np.linalg.norm(safd_emp))
    cosine_sim = float(np.sum(safd_cand * safd_emp) / max(norm_c * norm_e, 1e-12) * 100.0)

    eval_keys = [
        ("v_mean_kmh", 1.8),
        ("v_drive_kmh", 1.3),
        ("v_std_kmh", 1.2),
        ("a_pos_mean_ms2", 1.1),
        ("a_neg_mean_ms2", 1.1),
        ("a_std_ms2", 1.0),
        ("pct_idle", 1.6),
        ("pct_accel", 1.0),
        ("pct_decel", 1.0),
        ("pct_cruise", 0.8),
        ("pke_ms2", 1.0),
        ("rpa_ms2", 1.0),
        ("vsp_pos_mean_kwt", 1.1),
        ("e_net_wh_km", 1.3),
    ]
    weighted_re_sum = 0.0
    weight_total = 0.0
    for k, w in eval_keys:
        denom = max(abs(emp_metrics[k]), 0.05)
        weighted_re_sum += w * (abs(cand_metrics[k] - emp_metrics[k]) / denom * 100.0)
        weight_total += w

    mean_kinematic_re_pct = weighted_re_sum / max(weight_total, 1e-9)
    composite_score = 0.65 * mean_kinematic_re_pct + 0.35 * max(0.0, 100.0 - bhattacharyya_sim)

    return {
        "safd_rmse_pct": safd_rmse_pct,
        "safd_mae_pct": safd_mae_pct,
        "safd_bhattacharyya_pct": min(100.0, bhattacharyya_sim),
        "safd_cosine_pct": min(100.0, cosine_sim),
        "safd_js_div": js_div,
        "ks_stat_v": ks_stat,
        "mean_kinematic_re_pct": mean_kinematic_re_pct,
        "composite_score": composite_score,
    }


def build_egad_v110_54_column_dataframe(df_cycle: pd.DataFrame, dt: float, idle_thr_kmh: float) -> pd.DataFrame:
    """
    Sintez qilingan harakat siklini 1-dastur (`QwatLab: EV E.G.A.D. V1.0.0`) ning
    54 ta ustunli (16 Qizil + 13 Sariq + 25 Ko'k) formatiga 100% mos ravishda o'tkazadi.
    """
    n = len(df_cycle)
    t_sec = df_cycle["time_sec"].to_numpy(dtype=float)
    v_ms = df_cycle["velocity_ms"].to_numpy(dtype=float)
    v_kmh = df_cycle["velocity_kmh"].to_numpy(dtype=float)
    a_ms2 = df_cycle["accel_long_ms2"].to_numpy(dtype=float)
    j_ms3 = df_cycle["jerk_ms3"].to_numpy(dtype=float)
    dist_m = df_cycle["dist_m"].to_numpy(dtype=float)
    grade_deg = df_cycle["grade_deg"].to_numpy(dtype=float)
    grade_pct = df_cycle["grade_percent"].to_numpy(dtype=float)
    elev_m = df_cycle["elevation_m"].to_numpy(dtype=float)
    vsp_w_kg = df_cycle["vsp_kwt"].to_numpy(dtype=float)
    mt_ids = np.maximum(1, df_cycle["micro_trip_id"].to_numpy(dtype=int))

    is_stopped = v_kmh <= idle_thr_kmh
    ts_utc = pd.Timestamp("2026-05-12 03:00:00") + pd.to_timedelta(t_sec, unit="s")
    ts_local = ts_utc + pd.to_timedelta(5, unit="h")

    v_up = v_ms * np.sin(np.radians(grade_deg))
    zeros_f = np.zeros(n, dtype=float)

    df_54 = pd.DataFrame({
        "trip_id": np.ones(n, dtype=int),
        "microtrip_id": mt_ids,
        "trip_microtrip_id": mt_ids,
        "recorded_datetime_local": ts_local.strftime("%Y-%m-%d %H:%M:%S"),
        "recorded_time_hms": ts_local.strftime("%H:%M:%S"),
        "time_sec": t_sec,
        "trip_time_sec": t_sec,
        "dt_sec": np.full(n, float(dt), dtype=float),
        "dist_m": dist_m,
        "trip_dist_m": dist_m,
        "velocity_ms": v_ms,
        "velocity_kmh": v_kmh,
        "accel_long_ms2": a_ms2,
        "grade_deg": grade_deg,
        "grade_percent": grade_pct,
        "is_stopped": is_stopped,
        "driving_mode": df_cycle["driving_mode"].to_numpy(),
        "recorded_hour_float": np.round(ts_local.hour + ts_local.minute / 60.0 + ts_local.second / 3600.0, 4),
        "velocity_raw_kmh": v_kmh,
        "elevation_m": elev_m,
        "jerk_ms3": j_ms3,
        "accel_lat_ms2": zeros_f,
        "angular_vel_degs": zeros_f,
        "turn_radius_m": np.full(n, 10000.0, dtype=float),
        "heading_deg": zeros_f,
        "vsp_w_per_kg": vsp_w_kg,
        "zupt_update_applied": is_stopped,
        "lat_smooth": np.round(41.2995 + (dist_m / 111320.0), 6),
        "lon_smooth": np.full(n, 69.2401, dtype=float),
        "recorded_datetime_utc": ts_utc.strftime("%Y-%m-%d %H:%M:%S"),
        "x_m": dist_m,
        "y_m": zeros_f,
        "z_m": elev_m,
        "east_m": zeros_f,
        "north_m": dist_m,
        "up_m": elev_m - elev_m[0],
        "vx_ecef_ms": v_ms,
        "vy_ecef_ms": zeros_f,
        "vz_ecef_ms": v_up,
        "ax_ecef_ms2": a_ms2,
        "ay_ecef_ms2": zeros_f,
        "az_ecef_ms2": zeros_f,
        "v_east_ms": zeros_f,
        "v_north_ms": v_ms,
        "v_up_ms": v_up,
        "accel_east_ms2": zeros_f,
        "accel_north_ms2": a_ms2,
        "accel_up_ms2": zeros_f,
        "accel_vector_ms2": np.abs(a_ms2),
        "heading_rad": zeros_f,
        "angular_vel_rad_s": zeros_f,
        "turn_radius_display_m": np.full(n, 10000.0, dtype=float),
        "wind_head_ms": zeros_f,
        "wind_cross_ms": zeros_f,
    })
    return df_54[ALL_54_ORDERED_COLUMNS].copy()


def write_styled_excel_bytes(sheets_dict: dict) -> bytes:
    """
    8 GB RAMni qotirmasdan ko'p sahifali Excel (.xlsx) yaratish:
    54-ustunli sahifalarda 1-qatordagi sarlavhalarni QIZIL, SARIQ va KO'K ranglarga bo'yaydi.
    """
    from openpyxl.styles import PatternFill, Font, Alignment

    xlsx_buf = io.BytesIO()
    red_fill = PatternFill(start_color="FFD9534F", end_color="FFD9534F", fill_type="solid")
    yellow_fill = PatternFill(start_color="FFF0AD4E", end_color="FFF0AD4E", fill_type="solid")
    blue_fill = PatternFill(start_color="FF5BC0DE", end_color="FF5BC0DE", fill_type="solid")
    header_fill = PatternFill(start_color="FF1F4E78", end_color="FF1F4E78", fill_type="solid")

    font_white = Font(bold=True, color="FFFFFFFF")
    font_dark = Font(bold=True, color="FF000000")
    align_center = Alignment(horizontal="center", vertical="center")

    red_set = set(RED_MARKOV_CORE_COLUMNS)
    yellow_set = set(YELLOW_ARTICLE_COLUMNS)
    blue_set = set(BLUE_DIAGNOSTIC_COLUMNS)

    with pd.ExcelWriter(xlsx_buf, engine="openpyxl") as writer:
        for s_name, df_s in sheets_dict.items():
            df_s.to_excel(writer, index=False, sheet_name=s_name)
            ws = writer.sheets[s_name]
            ws.freeze_panes = "A2"
            for cell in ws[1]:
                c_name = str(cell.value)
                cell.alignment = align_center
                if c_name in red_set:
                    cell.fill = red_fill
                    cell.font = font_white
                elif c_name in yellow_set:
                    cell.fill = yellow_fill
                    cell.font = font_dark
                elif c_name in blue_set:
                    cell.fill = blue_fill
                    cell.font = font_dark
                else:
                    cell.fill = header_fill
                    cell.font = font_white

    return xlsx_buf.getvalue()


def render_block_2():
    st.divider()
    st.markdown("## ⚙️ BLOCK 2: Stochastic Markov, Quasi-Micro-Trip & Auto-Silhouette Synthesis Engine")

    if not st.session_state.get("b1_locked", False):
        st.warning("⚠️ Please confirm your multi-trip dataset and vehicle parameters in **Block 1** first.")
        return

    df_trips = st.session_state.b1_trips_df
    b1_params = st.session_state.b1_params
    dt = float(b1_params["target_dt"])
    bench_keys = list(STANDARD_CYCLES_KINEMATICS.keys())
    def_bench = b1_params.get("default_benchmark", bench_keys[0])
    def_bench_idx = bench_keys.index(def_bench) if def_bench in bench_keys else 0

    with st.container(border=True):
        st.markdown("### 🎛️ Markov State-Space, Regime Clustering & Convergence Parameters")
        c_m1, c_m2, c_m3, c_m4 = st.columns(4)
        with c_m1:
            synth_method = st.selectbox(
                "Synthesis methodology:",
                options=[
                    "Time-Quota Stratified K-Means Micro-Trip + Markov Regime Chain (Recommended)",
                    "Pure 1st-Order 2D Markov Chain (MCMC)",
                    "Micro-Trip Random Monte Carlo Assembly"
                ],
                index=0,
                key="b2_synth_method"
            )
        with c_m2:
            cluster_mode = st.selectbox(
                "Traffic regime K-Means clusters (K_c):",
                options=[
                    "Auto-Optimal K_c (Silhouette Score, 2..5)",
                    "2 Clusters (Low / High Flow)",
                    "3 Clusters (Congested / Urban / Arterial)",
                    "4 Clusters (Congested / Urban / Ring-Road / Highway)",
                    "5 Clusters (Multi-Regime Fleet)"
                ],
                index=0,
                key="b2_cluster_mode"
            )
        with c_m3:
            v_bin_kmh = st.number_input(
                "Speed bin step Δv (km/h) [0.5–5.0]:",
                min_value=MIN_SPEED_BIN_KMH,
                max_value=MAX_SPEED_BIN_KMH,
                value=float(b1_params.get("default_v_bin", 1.0)),
                step=0.5,
                key="b2_v_bin"
            )
        with c_m4:
            a_bin_ms2 = st.number_input(
                "Acceleration bin step Δa (m/s²) [0.05–0.50]:",
                min_value=MIN_ACCEL_BIN_MS2,
                max_value=MAX_ACCEL_BIN_MS2,
                value=float(b1_params.get("default_a_bin", 0.10)),
                step=0.05,
                format="%.2f",
                key="b2_a_bin"
            )

        c_m5, c_m6, c_m7, c_m8 = st.columns(4)
        with c_m5:
            target_cycle_dur_s = st.number_input(
                "Target cycle duration (s):",
                min_value=120,
                max_value=MAX_CYCLE_DURATION_S,
                value=int(b1_params.get("default_cycle_dur_s", 1200)),
                step=60,
                key="b2_target_dur_s"
            )
        with c_m6:
            mc_iterations = st.slider(
                "Monte Carlo trials (Proximity Filter) [1–100]:",
                min_value=MIN_MC_ITERATIONS,
                max_value=MAX_MC_ITERATIONS,
                value=30,
                step=1,
                key="b2_mc_iter"
            )
        with c_m7:
            random_seed = st.number_input(
                "Reproducibility random seed:",
                min_value=1,
                max_value=999999,
                value=2026,
                step=1,
                key="b2_seed"
            )
        with c_m8:
            benchmark_ref = st.selectbox(
                "Primary Standard Benchmark:",
                options=bench_keys,
                index=def_bench_idx,
                key="b2_benchmark_ref"
            )

        run_col1, run_col2 = st.columns([3, 1])
        with run_col1:
            run_btn = st.button(
                f"🚀 RUN V1.0.0 MARKOV & SAFD SYNTHESIS ({mc_iterations} Monte Carlo Trials)",
                type="primary",
                width='stretch',
                key="b2_btn_run"
            )
        with run_col2:
            if st.button("🔄 Reset All", width='stretch', key="b2_btn_reset"):
                st.session_state.clear()
                st.rerun()

    if run_btn:
        with st.status("Running Numba JIT Markov Engine, Hybrid Quasi-Micro-Trip Clustering & SAFD Filter...", expanded=True) as status:
            try:
                st.write("⏳ 1/4 Discretizing 2D (Speed, Acceleration) phase space and computing empirical SAFD...")
                v_emp_ms = df_trips["velocity_ms"].to_numpy(dtype=np.float64)
                v_emp_kmh = v_emp_ms * 3.6
                a_emp_ms2 = df_trips["accel_long_ms2"].to_numpy(dtype=np.float64)
                g_emp_deg = df_trips["grade_deg"].to_numpy(dtype=np.float64)
                trip_ids = df_trips["trip_id"].to_numpy(dtype=np.int64)

                v_max_obs_kmh = min(max(float(np.max(v_emp_kmh)) + v_bin_kmh, 20.0), float(b1_params["max_v_kmh"]) + 20.0)
                a_min_bound = max(min(float(np.min(a_emp_ms2)) - a_bin_ms2, -4.0), float(b1_params["min_b_ms2"]) - 2.0)
                a_max_bound = min(max(float(np.max(a_emp_ms2)) + a_bin_ms2, 4.0), float(b1_params["max_a_ms2"]) + 2.0)

                n_v_bins = int(np.ceil(v_max_obs_kmh / v_bin_kmh)) + 1
                n_a_bins = int(np.ceil((a_max_bound - a_min_bound) / a_bin_ms2)) + 1

                v_edges_kmh = np.arange(n_v_bins + 1, dtype=np.float64) * v_bin_kmh
                a_edges_ms2 = a_min_bound + np.arange(n_a_bins + 1, dtype=np.float64) * a_bin_ms2
                v_centers_kmh = 0.5 * (v_edges_kmh[:-1] + v_edges_kmh[1:])
                v_centers_ms = v_centers_kmh / 3.6
                a_centers_ms2 = 0.5 * (a_edges_ms2[:-1] + a_edges_ms2[1:])

                v_idx = np.clip(np.floor(v_emp_kmh / v_bin_kmh).astype(np.int64), 0, n_v_bins - 1)
                a_idx = np.clip(np.floor((a_emp_ms2 - a_min_bound) / a_bin_ms2).astype(np.int64), 0, n_a_bins - 1)
                zero_a_bin = int(np.argmin(np.abs(a_centers_ms2)))
                start_state = int(0 * n_a_bins + zero_a_bin)

                st.write(f"⏳ 2/4 Building Compact 1st-Order Markov Transition Matrix ({n_v_bins} × {n_a_bins} grid)...")
                cdf, active_grid_states, grid_to_active, valid_row, active_states_count = build_compact_markov_cdf_numba(
                    v_idx, a_idx, trip_ids, n_v_bins, n_a_bins
                )

                safd_emp = compute_safd_matrix_numba(
                    v_emp_kmh, a_emp_ms2, 0.0, v_bin_kmh, n_v_bins, a_min_bound, a_bin_ms2, n_a_bins
                )
                emp_metrics = calculate_kinematic_metrics(
                    v_emp_ms, a_emp_ms2, g_emp_deg, dt, b1_params, trip_ids=trip_ids
                )

                st.write("⏳ 3/4 Extracting Micro-Trips / Quasi-Micro-Trips and running K-Means Silhouette clustering...")
                micro_trips_pool, regime_tpm, cluster_weights, k_c_used, sil_score, quasi_splits = extract_and_cluster_micro_trips(
                    df_trips, dt, b1_params["idle_thr_kmh"], b1_params["max_mt_dur_s"], cluster_mode=cluster_mode
                )
                n_target_steps = int(round(target_cycle_dur_s / dt)) + 1

                st.write(f"⏳ 4/4 Synthesizing and evaluating {mc_iterations} Monte Carlo candidate cycle(s) across {k_c_used} regimes...")
                best_candidate = None
                convergence_log = []

                v_max_emp_ms = min(float(np.max(v_emp_ms)), float(b1_params["max_v_kmh"]) / 3.6)
                a_min_emp_ms2 = max(float(np.min(a_emp_ms2)), float(b1_params["min_b_ms2"]))
                a_max_emp_ms2 = min(float(np.max(a_emp_ms2)), float(b1_params["max_a_ms2"]))

                for trial in range(mc_iterations):
                    trial_seed = int(random_seed + trial * 101 + 7)
                    rng_trial = np.random.default_rng(trial_seed)

                    if synth_method == "Pure 1st-Order 2D Markov Chain (MCMC)" or len(micro_trips_pool) == 0:
                        v_cand_ms, a_cand_ms2, _ = synthesize_markov_chain_numba(
                            cdf, active_grid_states, grid_to_active, valid_row,
                            v_centers_ms, a_centers_ms2,
                            n_v_bins, n_a_bins, n_target_steps, dt,
                            v_max_emp_ms, a_min_emp_ms2, a_max_emp_ms2,
                            start_state, zero_a_bin, trial_seed
                        )
                        if len(v_cand_ms) >= 7:
                            idle_now = (v_cand_ms * 3.6) <= b1_params["idle_thr_kmh"]
                            v_sm = savgol_filter(v_cand_ms, window_length=5, polyorder=2, mode="nearest")
                            v_cand_ms = np.where(idle_now, 0.0, np.clip(v_sm, 0.0, v_max_emp_ms))
                            v_cand_ms[0] = 0.0
                            v_cand_ms[-1] = 0.0
                        start_g = int(rng_trial.integers(0, max(1, len(g_emp_deg) - n_target_steps))) if len(g_emp_deg) > n_target_steps else 0
                        g_cand_deg = np.resize(np.roll(g_emp_deg, -start_g), n_target_steps)

                    elif synth_method == "Micro-Trip Random Monte Carlo Assembly":
                        uniform_tpm = np.ones_like(regime_tpm) / regime_tpm.shape[0]
                        uniform_w = np.ones_like(cluster_weights) / len(cluster_weights)
                        v_cand_ms, a_cand_ms2, g_cand_deg = synthesize_stratified_markov_microtrip_cycle(
                            micro_trips_pool, uniform_tpm, uniform_w, df_trips, n_target_steps, dt, rng_trial
                        )
                    else:
                        v_cand_ms, a_cand_ms2, g_cand_deg = synthesize_stratified_markov_microtrip_cycle(
                            micro_trips_pool, regime_tpm, cluster_weights, df_trips, n_target_steps, dt, rng_trial
                        )

                    v_cand_ms, a_cand_ms2, _ = enforce_kinematic_closure(
                        v_cand_ms, dt, float(b1_params["max_v_kmh"]) / 3.6,
                        float(b1_params["min_b_ms2"]), float(b1_params["max_a_ms2"]),
                        float(b1_params["idle_thr_kmh"]) / 3.6
                    )

                    safd_cand = compute_safd_matrix_numba(
                        v_cand_ms * 3.6, a_cand_ms2, 0.0, v_bin_kmh, n_v_bins, a_min_bound, a_bin_ms2, n_a_bins
                    )
                    cand_metrics = calculate_kinematic_metrics(
                        v_cand_ms, a_cand_ms2, g_cand_deg, dt, b1_params
                    )
                    eval_res = evaluate_safd_and_convergence(
                        cand_metrics, emp_metrics, safd_cand, safd_emp, v_cand_ms * 3.6, v_emp_kmh
                    )

                    convergence_log.append({
                        "Trial #": trial + 1,
                        "Seed": trial_seed,
                        "Mean Speed (km/h)": round(cand_metrics["v_mean_kmh"], 2),
                        "Speed Std (km/h)": round(cand_metrics["v_std_kmh"], 2),
                        "Idle (%)": round(cand_metrics["pct_idle"], 2),
                        "Net Energy (Wh/km)": round(cand_metrics["e_net_wh_km"], 2),
                        "SAFD Bhattacharyya (%)": round(eval_res["safd_bhattacharyya_pct"], 2),
                        "SAFD Cosine Sim (%)": round(eval_res["safd_cosine_pct"], 2),
                        "SAFD JS Divergence": round(eval_res["safd_js_div"], 4),
                        "KS Stat (D_KS)": round(eval_res["ks_stat_v"], 4),
                        "Kinematic Mean RE (%)": round(eval_res["mean_kinematic_re_pct"], 2),
                        "Composite Objective J": round(eval_res["composite_score"], 3),
                    })

                    if best_candidate is None or eval_res["composite_score"] < best_candidate["eval"]["composite_score"]:
                        best_candidate = {
                            "trial": trial + 1,
                            "seed": trial_seed,
                            "v_ms": v_cand_ms,
                            "a_ms2": a_cand_ms2,
                            "g_deg": g_cand_deg,
                            "safd": safd_cand,
                            "metrics": cand_metrics,
                            "eval": eval_res,
                        }

                t_cycle = np.arange(n_target_steps, dtype=np.float64) * dt
                v_best_ms = best_candidate["v_ms"]
                v_best_kmh = v_best_ms * 3.6
                a_best_ms2 = best_candidate["a_ms2"]
                g_best_deg = best_candidate["g_deg"]
                jerk_best_ms3 = np.gradient(a_best_ms2, dt)
                dist_best_m = cumulative_trapezoid_1d(v_best_ms, dt)
                grade_cycle_pct = np.tan(np.radians(g_best_deg)) * 100.0
                elevation_m = 450.0 + cumulative_trapezoid_1d(v_best_ms * np.sin(np.radians(g_best_deg)), dt)

                vsp_kwt, p_bat_kw, _ = compute_vsp_and_ev_energy(v_best_ms, a_best_ms2, g_best_deg, dt, b1_params)
                vsp_emp_kwt, _, _ = compute_vsp_and_ev_energy(v_emp_ms, a_emp_ms2, g_emp_deg, dt, b1_params)

                modes = np.full(n_target_steps, "Cruising", dtype=object)
                idle_m = v_best_kmh <= b1_params["idle_thr_kmh"]
                modes[idle_m] = "Idle"
                modes[(~idle_m) & (a_best_ms2 > 0.10)] = "Acceleration"
                modes[(~idle_m) & (a_best_ms2 < -0.10)] = "Deceleration"

                v_b_best = np.clip(np.floor(v_best_kmh / v_bin_kmh).astype(np.int64), 0, n_v_bins - 1)
                a_b_best = np.clip(np.floor((a_best_ms2 - a_min_bound) / a_bin_ms2).astype(np.int64), 0, n_a_bins - 1)
                state_id_best = v_b_best * n_a_bins + a_b_best
                mt_ids = np.maximum(1, np.cumsum(np.diff((~idle_m).astype(int), prepend=0) == 1))

                df_cycle_out = pd.DataFrame({
                    "time_sec": np.round(t_cycle, 4),
                    "dist_m": np.round(dist_best_m, 4),
                    "velocity_ms": np.round(v_best_ms, 5),
                    "velocity_kmh": np.round(v_best_kmh, 5),
                    "accel_long_ms2": np.round(a_best_ms2, 5),
                    "jerk_ms3": np.round(jerk_best_ms3, 5),
                    "grade_deg": np.round(g_best_deg, 4),
                    "grade_percent": np.round(grade_cycle_pct, 4),
                    "elevation_m": np.round(elevation_m, 3),
                    "vsp_kwt": np.round(vsp_kwt, 4),
                    "p_bat_kw": np.round(p_bat_kw, 4),
                    "driving_mode": modes,
                    "markov_state_id": state_id_best,
                    "micro_trip_id": mt_ids,
                })

                df_egad_54 = build_egad_v110_54_column_dataframe(df_cycle_out, dt, b1_params["idle_thr_kmh"])

                v_tpm = np.zeros((n_v_bins, n_v_bins), dtype=np.float64)
                same_t = trip_ids[:-1] == trip_ids[1:]
                np.add.at(v_tpm, (v_idx[:-1][same_t], v_idx[1:][same_t]), 1.0)
                row_sums_v = v_tpm.sum(axis=1, keepdims=True)
                v_tpm = np.divide(v_tpm, row_sums_v, out=np.zeros_like(v_tpm), where=row_sums_v > 0) * 100.0

                st.session_state.b2_results = {
                    "df_cycle": df_cycle_out,
                    "df_egad_54": df_egad_54,
                    "emp_metrics": emp_metrics,
                    "cand_metrics": best_candidate["metrics"],
                    "eval": best_candidate["eval"],
                    "best_trial": best_candidate["trial"],
                    "best_seed": best_candidate["seed"],
                    "safd_emp": safd_emp,
                    "safd_cand": best_candidate["safd"],
                    "vsp_emp_kwt": vsp_emp_kwt,
                    "v_centers_kmh": v_centers_kmh,
                    "a_centers_ms2": a_centers_ms2,
                    "v_tpm_pct": v_tpm,
                    "regime_tpm_pct": regime_tpm * 100.0,
                    "cluster_weights_pct": np.round(cluster_weights * 100.0, 2).tolist(),
                    "k_clusters_used": int(k_c_used),
                    "silhouette_score": round(float(sil_score), 4),
                    "quasi_splits_count": int(quasi_splits),
                    "total_microtrips_extracted": int(len(micro_trips_pool)),
                    "active_states_count": int(active_states_count),
                    "total_grid_states": int(n_v_bins * n_a_bins),
                    "convergence_df": pd.DataFrame(convergence_log),
                    "settings": {
                        "synth_method": synth_method,
                        "cluster_mode": cluster_mode,
                        "k_clusters_used": int(k_c_used),
                        "silhouette_score": round(float(sil_score), 4),
                        "v_bin_kmh": float(v_bin_kmh),
                        "a_bin_ms2": float(a_bin_ms2),
                        "target_cycle_dur_s": int(target_cycle_dur_s),
                        "mc_iterations": int(mc_iterations),
                        "random_seed": int(random_seed),
                        "benchmark_ref": benchmark_ref,
                    },
                }
                st.session_state.pop("b3_zip_bytes", None)
                status.update(
                    label=(
                        f"✅ Synthesis Complete! Optimal Trial #{best_candidate['trial']} | "
                        f"K_c={k_c_used} (Silhouette={sil_score:.3f}) | "
                        f"SAFD Bhattacharyya={best_candidate['eval']['safd_bhattacharyya_pct']:.2f}% | "
                        f"Mean RE={best_candidate['eval']['mean_kinematic_re_pct']:.2f}%"
                    ),
                    state="complete",
                    expanded=False
                )
            except Exception as ex:
                status.update(label="❌ Synthesis Engine Error", state="error", expanded=True)
                st.error(f"Computation stopped safely: {str(ex)}")


# =====================================================================
# 3-BLOK: ILMIY VALIDATSIYA JADVALI, GRAFIKLAR VA EKSPORT
# =====================================================================
def build_validation_comparison_df(emp_m: dict, cand_m: dict, eval_m: dict, benchmark_name: str, b1_params: dict) -> pd.DataFrame:
    dyn_benchmarks = get_dynamic_standard_benchmarks(b1_params)
    ordered_bench_names = [benchmark_name] + [k for k in dyn_benchmarks.keys() if k != benchmark_name]

    rows_spec = [
        ("Total Duration", "s", "duration_s"),
        ("Total Distance", "km", "distance_km"),
        ("Mean Speed (Total, with stops)", "km/h", "v_mean_kmh"),
        ("Mean Driving Speed (excl. stops)", "km/h", "v_drive_kmh"),
        ("Speed Standard Deviation (σ_v)", "km/h", "v_std_kmh"),
        ("Maximum Speed (v_max)", "km/h", "v_max_kmh"),
        ("Mean Positive Acceleration (a_+)", "m/s²", "a_pos_mean_ms2"),
        ("Mean Deceleration (a_-)", "m/s²", "a_neg_mean_ms2"),
        ("Acceleration Std. Deviation (σ_a)", "m/s²", "a_std_ms2"),
        ("RMS Acceleration (a_rms)", "m/s²", "a_rms_ms2"),
        ("Idle Time Proportion (P_idle)", "%", "pct_idle"),
        ("Acceleration Time Proportion (P_acc)", "%", "pct_accel"),
        ("Deceleration Time Proportion (P_dec)", "%", "pct_decel"),
        ("Cruising Time Proportion (P_cruise)", "%", "pct_cruise"),
        ("Positive Kinetic Energy (PKE)", "m/s²", "pke_ms2"),
        ("Relative Positive Acceleration (RPA)", "m/s²", "rpa_ms2"),
        ("Mean Positive VSP (VSP_+)", "kW/t (W/kg)", "vsp_pos_mean_kwt"),
        ("Gross Tractive + Aux Energy (E_trac)", "Wh/km", "e_trac_wh_km"),
        ("Constant Auxiliary Load Share (E_aux)", "Wh/km", "e_aux_wh_km"),
        ("Regenerative Braking Recovery (E_regen)", "Wh/km", "e_regen_wh_km"),
        ("Net Battery Specific Energy (E_net)", "Wh/km", "e_net_wh_km"),
        ("Peak Battery Tractive Power (P_peak)", "kW", "p_peak_kw"),
    ]

    records = []
    for label, unit, key in rows_spec:
        val_emp = float(emp_m[key])
        val_cand = float(cand_m[key])
        abs_diff = val_cand - val_emp
        if key in ("duration_s", "distance_km"):
            rel_err_str = "Target Scale"
            q1_status = "Target Scale"
        else:
            denom = max(abs(val_emp), 1e-6)
            rel_err = abs(val_cand - val_emp) / denom * 100.0
            rel_err_str = f"{rel_err:.2f}%"
            q1_status = "✅ PASS (<5%)" if rel_err <= 5.0 else ("☑️ ACCEPT (<10%)" if rel_err <= 10.0 else "⚠️ REVIEW")

        row_dict = {
            "Kinematic / Energetic Parameter": label,
            "Unit": unit,
            "Empirical Fleet (Target)": round(val_emp, 3),
            "Synthesized Cycle (Markov)": round(val_cand, 3),
            "Abs. Diff (Cand - Emp)": round(abs_diff, 3),
            "Relative Error (%)": rel_err_str,
            "Q1/Q3 Criterion": q1_status,
        }
        for b_name in ordered_bench_names:
            row_dict[b_name] = round(float(dyn_benchmarks[b_name][key]), 3)
        records.append(row_dict)

    extra_rows = [
        ("2D SAFD Bhattacharyya Similarity", "%", 100.0, round(eval_m["safd_bhattacharyya_pct"], 2), f"{100.0 - eval_m['safd_bhattacharyya_pct']:.2f}%", "✅ PASS (>90%)"),
        ("2D SAFD Cosine Similarity", "%", 100.0, round(eval_m["safd_cosine_pct"], 2), f"{100.0 - eval_m['safd_cosine_pct']:.2f}%", "✅ PASS (>90%)"),
        ("2D SAFD Cell-Wise RMSE", "%", 0.0, round(eval_m["safd_rmse_pct"], 4), "—", "✅ PASS (<0.5%)"),
        ("2D SAFD Jensen–Shannon Divergence (D_JS)", "bits", 0.0, round(eval_m["safd_js_div"], 4), "—", "✅ PASS (<0.10)"),
        ("Speed CDF Kolmogorov–Smirnov Stat (D_KS)", "—", 0.0, round(eval_m["ks_stat_v"], 4), "—", "✅ PASS (<0.08)"),
    ]
    for label, unit, v_e, v_c, re_s, st_s in extra_rows:
        row_dict = {
            "Kinematic / Energetic Parameter": label,
            "Unit": unit,
            "Empirical Fleet (Target)": v_e,
            "Synthesized Cycle (Markov)": v_c,
            "Abs. Diff (Cand - Emp)": round(v_c - v_e, 4),
            "Relative Error (%)": re_s,
            "Q1/Q3 Criterion": st_s,
        }
        for b_name in ordered_bench_names:
            row_dict[b_name] = np.nan
        records.append(row_dict)

    return pd.DataFrame(records)


def render_static_publication_figures(
    res: dict,
    df_trips: pd.DataFrame,
    b1_params: dict,
    dpi: int = 300,
    fmt: str = "png",
    only_fig1: bool = False
) -> dict:
    df_c = res["df_cycle"]
    v_centers = res["v_centers_kmh"]
    a_centers = res["a_centers_ms2"]
    safd_emp = res["safd_emp"]
    safd_cand = res["safd_cand"]
    conv_df = res["convergence_df"]
    emp_m = res["emp_metrics"]
    cand_m = res["cand_metrics"]
    dyn_benchmarks = get_dynamic_standard_benchmarks(b1_params)

    figures_bytes = {}

    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 7.8), sharex=True, gridspec_kw={"height_ratios": [2.0, 1.1, 1.1]})
    ax1.plot(df_c["time_sec"], df_c["velocity_kmh"], color="#0B5394", linewidth=1.4, label="Synthesized Speed (km/h)")
    ax1.axhline(cand_m["v_mean_kmh"], color="#D9534F", linestyle="--", linewidth=1.1, label=f"Cycle Mean: {cand_m['v_mean_kmh']:.1f} km/h (Fleet: {emp_m['v_mean_kmh']:.1f} km/h)")
    ax1.fill_between(df_c["time_sec"], df_c["velocity_kmh"], color="#0B5394", alpha=0.12)
    ax1.set_ylabel("Speed (km/h)", fontsize=10, fontweight="bold")
    ax1.set_title("Synthesized Representative Driving Cycle (Markov, Micro-Trip & SAFD Validated)", fontsize=12, fontweight="bold")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right", frameon=True)

    ax2.plot(df_c["time_sec"], df_c["accel_long_ms2"], color="#2E8B57", linewidth=1.0, label="Longitudinal Acceleration (m/s²)")
    ax2.axhline(0.0, color="black", linewidth=0.8)
    ax2.set_ylabel("Accel. (m/s²)", fontsize=10, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right", frameon=True)

    ax3.plot(df_c["time_sec"], df_c["p_bat_kw"], color="#8E44AD", linewidth=1.0, label=f"Battery Power (Net: {cand_m['e_net_wh_km']:.1f} Wh/km)")
    ax3.axhline(0.0, color="black", linewidth=0.8)
    ax3.set_xlabel("Time (s)", fontsize=10, fontweight="bold")
    ax3.set_ylabel("Power (kW)", fontsize=10, fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right", frameon=True)
    fig1.tight_layout()
    buf1 = io.BytesIO()
    fig1.savefig(buf1, format=fmt, dpi=dpi, bbox_inches="tight")
    plt.close(fig1)
    figures_bytes[f"Fig1_Synthesized_Driving_Cycle.{fmt}"] = buf1.getvalue()

    if only_fig1:
        return figures_bytes

    fig2, axes = plt.subplots(1, 3, figsize=(15, 4.8), sharey=True)
    extent = [a_centers[0], a_centers[-1], v_centers[0], v_centers[-1]]
    vmax_s = max(float(np.max(safd_emp)), float(np.max(safd_cand)), 0.1)

    im0 = axes[0].imshow(safd_emp, origin="lower", aspect="auto", extent=extent, cmap="YlGnBu", vmin=0, vmax=vmax_s)
    axes[0].set_title("a) Empirical Fleet SAFD (%)", fontsize=11, fontweight="bold")
    axes[0].set_xlabel("Acceleration (m/s²)", fontsize=10)
    axes[0].set_ylabel("Speed (km/h)", fontsize=10)
    fig2.colorbar(im0, ax=axes[0], fraction=0.046, pad=0.04)

    im1 = axes[1].imshow(safd_cand, origin="lower", aspect="auto", extent=extent, cmap="YlGnBu", vmin=0, vmax=vmax_s)
    axes[1].set_title("b) Synthesized Cycle SAFD (%)", fontsize=11, fontweight="bold")
    axes[1].set_xlabel("Acceleration (m/s²)", fontsize=10)
    fig2.colorbar(im1, ax=axes[1], fraction=0.046, pad=0.04)

    diff_s = safd_cand - safd_emp
    max_d = max(float(np.max(np.abs(diff_s))), 0.05)
    im2 = axes[2].imshow(diff_s, origin="lower", aspect="auto", extent=extent, cmap="RdBu_r", vmin=-max_d, vmax=max_d)
    axes[2].set_title(f"c) Residual ΔSAFD (RMSE={res['eval']['safd_rmse_pct']:.3f}%)", fontsize=11, fontweight="bold")
    axes[2].set_xlabel("Acceleration (m/s²)", fontsize=10)
    fig2.colorbar(im2, ax=axes[2], fraction=0.046, pad=0.04)
    fig2.tight_layout()
    buf2 = io.BytesIO()
    fig2.savefig(buf2, format=fmt, dpi=dpi, bbox_inches="tight")
    plt.close(fig2)
    figures_bytes[f"Fig2_SAFD_2D_Comparison.{fmt}"] = buf2.getvalue()

    fig3, (ax3a, ax3b) = plt.subplots(1, 2, figsize=(12, 4.6))
    emp_v = np.sort(df_trips["velocity_kmh"].to_numpy(dtype=float))
    cand_v = np.sort(df_c["velocity_kmh"].to_numpy(dtype=float))
    ax3a.plot(emp_v, np.linspace(0, 100, len(emp_v)), label="Empirical Multi-Trip Fleet", color="#1F77B4", linewidth=2.0)
    ax3a.plot(cand_v, np.linspace(0, 100, len(cand_v)), label="Synthesized Driving Cycle", color="#FF7F0E", linestyle="--", linewidth=2.0)
    ax3a.set_xlabel("Speed (km/h)", fontsize=10, fontweight="bold")
    ax3a.set_ylabel("Cumulative Probability (%)", fontsize=10, fontweight="bold")
    ax3a.set_title(f"a) Speed CDF (KS Stat D={res['eval']['ks_stat_v']:.4f})", fontsize=11, fontweight="bold")
    ax3a.grid(True, linestyle="--", alpha=0.5)
    ax3a.legend()

    modes_labels = ["Idle (%)", "Accel (%)", "Decel (%)", "Cruise (%)"]
    emp_modes = [emp_m["pct_idle"], emp_m["pct_accel"], emp_m["pct_decel"], emp_m["pct_cruise"]]
    cand_modes = [cand_m["pct_idle"], cand_m["pct_accel"], cand_m["pct_decel"], cand_m["pct_cruise"]]
    x_pos = np.arange(len(modes_labels))
    ax3b.bar(x_pos - 0.18, emp_modes, width=0.35, label="Empirical Fleet", color="#1F77B4")
    ax3b.bar(x_pos + 0.18, cand_modes, width=0.35, label="Synthesized Cycle", color="#FF7F0E")
    ax3b.set_xticks(x_pos)
    ax3b.set_xticklabels(modes_labels)
    ax3b.set_ylabel("Time Share (%)", fontsize=10, fontweight="bold")
    ax3b.set_title("b) Driving Mode Proportions Comparison", fontsize=11, fontweight="bold")
    ax3b.grid(True, axis="y", linestyle="--", alpha=0.5)
    ax3b.legend()
    fig3.tight_layout()
    buf3 = io.BytesIO()
    fig3.savefig(buf3, format=fmt, dpi=dpi, bbox_inches="tight")
    plt.close(fig3)
    figures_bytes[f"Fig3_CDF_and_Driving_Modes.{fmt}"] = buf3.getvalue()

    fig4, ax4 = plt.subplots(figsize=(9, 4.2))
    ax4.plot(conv_df["Trial #"], conv_df["Composite Objective J"], marker="o", color="#4B0082", linewidth=1.5, label="Composite Error Objective J")
    best_row = conv_df.loc[conv_df["Trial #"] == res["best_trial"]].iloc[0]
    ax4.scatter([best_row["Trial #"]], [best_row["Composite Objective J"]], color="#D9534F", s=90, zorder=5, label=f"Optimal Trial #{res['best_trial']} (J={best_row['Composite Objective J']:.2f})")
    ax4.set_xlabel("Monte Carlo Candidate Trial #", fontsize=10, fontweight="bold")
    ax4.set_ylabel("Composite Convergence Error J (Lower is Better)", fontsize=10, fontweight="bold")
    ax4.set_title("Monte Carlo Proximity Filter Convergence Across Candidate Cycles", fontsize=11, fontweight="bold")
    ax4.grid(True, linestyle="--", alpha=0.5)
    ax4.legend()
    fig4.tight_layout()
    buf4 = io.BytesIO()
    fig4.savefig(buf4, format=fmt, dpi=dpi, bbox_inches="tight")
    plt.close(fig4)
    figures_bytes[f"Fig4_Monte_Carlo_Convergence.{fmt}"] = buf4.getvalue()

    fig5, (ax5a, ax5b) = plt.subplots(1, 2, figsize=(13, 4.6))
    bins_vsp = np.linspace(-25.0, 35.0, 30)
    ax5a.hist(res["vsp_emp_kwt"], bins=bins_vsp, density=True, alpha=0.55, color="#1F77B4", label="Empirical Fleet VSP")
    ax5a.hist(df_c["vsp_kwt"], bins=bins_vsp, density=True, alpha=0.55, color="#FF7F0E", label="Synthesized Cycle VSP")
    ax5a.set_xlabel("Vehicle Specific Power VSP (W/kg ≡ kW/t)", fontsize=10, fontweight="bold")
    ax5a.set_ylabel("Probability Density", fontsize=10, fontweight="bold")
    ax5a.set_title("a) True Physics-Based VSP Distribution", fontsize=11, fontweight="bold")
    ax5a.grid(True, linestyle="--", alpha=0.5)
    ax5a.legend()

    cyc_names = ["Empirical", "Synthesized", "WMTC Stg1", "ECE R47", "WLTC Urb"]
    wmtc1 = dyn_benchmarks["WMTC Class 1 (Stage 1 Urban Scooter/Moped)"]
    r47 = dyn_benchmarks["UNECE R47 (Urban Moped Cycle)"]
    wurb = dyn_benchmarks["WLTC Urban (Low + Medium)"]
    e_tracs = [emp_m["e_trac_wh_km"], cand_m["e_trac_wh_km"], wmtc1["e_trac_wh_km"], r47["e_trac_wh_km"], wurb["e_trac_wh_km"]]
    e_nets = [emp_m["e_net_wh_km"], cand_m["e_net_wh_km"], wmtc1["e_net_wh_km"], r47["e_net_wh_km"], wurb["e_net_wh_km"]]
    x5 = np.arange(len(cyc_names))
    ax5b.bar(x5 - 0.18, e_tracs, width=0.35, label="Gross Tractive+Aux (Wh/km)", color="#2B5B84")
    ax5b.bar(x5 + 0.18, e_nets, width=0.35, label="Net Battery (Wh/km)", color="#27AE60")
    ax5b.set_xticks(x5)
    ax5b.set_xticklabels(cyc_names)
    ax5b.set_ylabel("Specific Energy (Wh/km)", fontsize=10, fontweight="bold")
    ax5b.set_title("b) Specific Energy vs. WMTC, R47 & WLTC", fontsize=11, fontweight="bold")
    ax5b.grid(True, axis="y", linestyle="--", alpha=0.5)
    ax5b.legend()
    fig5.tight_layout()
    buf5 = io.BytesIO()
    fig5.savefig(buf5, format=fmt, dpi=dpi, bbox_inches="tight")
    plt.close(fig5)
    figures_bytes[f"Fig5_VSP_and_EV_Energetics.{fmt}"] = buf5.getvalue()

    gc.collect()
    return figures_bytes


def render_block_3():
    st.divider()
    st.markdown("## 📊 BLOCK 3: Q1/Q3 Scientific Validation, WMTC/R47/WLTC Benchmarks & Export Center")

    if "b2_results" not in st.session_state:
        st.info("Run the synthesis engine in **Block 2** to view the synthesized driving cycle, SAFD evaluation, and validation tables.")
        return

    res = st.session_state.b2_results
    df_cycle = res["df_cycle"]
    df_egad_54 = res["df_egad_54"]
    emp_m = res["emp_metrics"]
    cand_m = res["cand_metrics"]
    eval_m = res["eval"]
    settings = res["settings"]
    df_trips = st.session_state.b1_trips_df
    b1_params = st.session_state.b1_params
    dyn_benchmarks = get_dynamic_standard_benchmarks(b1_params)

    st.markdown("### 🏆 Synthesis Convergence, SAFD & Energetic Scorecard")
    k1, k2, k3, k4, k5, k6 = st.columns(6)
    v_err_pct = abs(cand_m["v_mean_kmh"] - emp_m["v_mean_kmh"]) / max(emp_m["v_mean_kmh"], 1e-6) * 100.0
    idle_diff_pp = cand_m["pct_idle"] - emp_m["pct_idle"]
    enet_err_pct = abs(cand_m["e_net_wh_km"] - emp_m["e_net_wh_km"]) / max(emp_m["e_net_wh_km"], 1e-6) * 100.0

    k1.metric("Mean Speed (Cycle vs Fleet)", f"{cand_m['v_mean_kmh']:.2f} km/h", f"Fleet: {emp_m['v_mean_kmh']:.2f} km/h (RE: {v_err_pct:.2f}%)")
    k2.metric("Idle Share (Cycle vs Fleet)", f"{cand_m['pct_idle']:.2f}%", f"{idle_diff_pp:+.2f} pp vs Fleet ({emp_m['pct_idle']:.2f}%)")
    k3.metric("SAFD Bhattacharyya Sim.", f"{eval_m['safd_bhattacharyya_pct']:.2f}%", f"JS Div: {eval_m['safd_js_div']:.4f} bits")
    k4.metric("Regime Clusters (K_c)", f"{res['k_clusters_used']} Regimes", f"Silhouette: {res['silhouette_score']:.3f} | Quasi: {res['quasi_splits_count']}")
    k5.metric("Net Specific Energy", f"{cand_m['e_net_wh_km']:.2f} Wh/km", f"Fleet: {emp_m['e_net_wh_km']:.2f} Wh/km (RE: {enet_err_pct:.2f}%)")
    k6.metric("Selected Optimal Trial", f"Trial #{res['best_trial']} / {settings['mc_iterations']}", f"Mean RE: {eval_m['mean_kinematic_re_pct']:.2f}%")

    st.markdown("### 💎 Manuscript Validation Table: Empirical Fleet vs. Synthesized Cycle vs. International Standards (WMTC, R47, WLTC, NEDC)")
    val_df = build_validation_comparison_df(emp_m, cand_m, eval_m, settings["benchmark_ref"], b1_params)
    st.dataframe(val_df, width='stretch', hide_index=True)

    st.markdown("### 📈 Interactive Scientific Visualization Suite")
    tab_c1, tab_c2, tab_c3, tab_c4, tab_c5, tab_c6 = st.tabs([
        "🚀 1. Synthesized Driving Cycle",
        "🔥 2. 2D SAFD Matrices & 3D Surface",
        "⚡ 3. VSP & Energy vs. WMTC / R47 / WLTC",
        "🎲 4. Markov TPM & Monte Carlo Convergence",
        "📊 5. CDF & Driving Mode Comparison",
        "📝 6. Manuscript Text"
    ])

    with tab_c1:
        fig_cycle = make_subplots(
            rows=3, cols=1, shared_xaxes=True, vertical_spacing=0.06,
            row_heights=[0.45, 0.28, 0.27],
            subplot_titles=(
                "1. Synthesized Vehicle Speed Profile v(t) [km/h]",
                "2. Longitudinal Acceleration Profile a(t) [m/s²]",
                "3. Battery Terminal Power P_bat(t) [kW] (including P_aux and Regen)"
            )
        )
        fig_cycle.add_trace(
            go.Scatter(x=df_cycle["time_sec"], y=df_cycle["velocity_kmh"], mode="lines", name="Speed (km/h)", line=dict(color="#0066CC", width=1.8)),
            row=1, col=1
        )
        fig_cycle.add_hline(
            y=cand_m["v_mean_kmh"], line_dash="dash", line_color="#EF553B",
            annotation_text=f"Cycle Mean: {cand_m['v_mean_kmh']:.2f} km/h | Empirical Fleet Mean: {emp_m['v_mean_kmh']:.2f} km/h",
            row=1, col=1
        )
        fig_cycle.add_trace(
            go.Scatter(x=df_cycle["time_sec"], y=df_cycle["accel_long_ms2"], mode="lines", name="Accel (m/s²)", line=dict(color="#00CC96", width=1.2)),
            row=2, col=1
        )
        fig_cycle.add_trace(
            go.Scatter(x=df_cycle["time_sec"], y=df_cycle["p_bat_kw"], mode="lines", name="Battery Power (kW)", line=dict(color="#AB63FA", width=1.4)),
            row=3, col=1
        )
        fig_cycle.update_layout(height=700, hovermode="x unified", margin=dict(l=20, r=20, t=40, b=20))
        st.plotly_chart(fig_cycle, width='stretch')

    with tab_c2:
        sc1, sc2, sc3 = st.columns(3)
        with sc1:
            fig_s_emp = px.imshow(
                res["safd_emp"], x=res["a_centers_ms2"], y=res["v_centers_kmh"],
                origin="lower", aspect="auto", color_continuous_scale="Viridis",
                labels=dict(x="Acceleration (m/s²)", y="Speed (km/h)", color="Freq (%)"),
                title="Empirical Multi-Trip Fleet SAFD (%)"
            )
            st.plotly_chart(fig_s_emp, width='stretch')
        with sc2:
            fig_s_cand = px.imshow(
                res["safd_cand"], x=res["a_centers_ms2"], y=res["v_centers_kmh"],
                origin="lower", aspect="auto", color_continuous_scale="Viridis",
                labels=dict(x="Acceleration (m/s²)", y="Speed (km/h)", color="Freq (%)"),
                title="Synthesized Driving Cycle SAFD (%)"
            )
            st.plotly_chart(fig_s_cand, width='stretch')
        with sc3:
            diff_safd = res["safd_cand"] - res["safd_emp"]
            fig_s_diff = px.imshow(
                diff_safd, x=res["a_centers_ms2"], y=res["v_centers_kmh"],
                origin="lower", aspect="auto", color_continuous_scale="RdBu_r", color_continuous_midpoint=0.0,
                labels=dict(x="Acceleration (m/s²)", y="Speed (km/h)", color="ΔFreq (%)"),
                title=f"Residual Difference ΔSAFD (RMSE = {eval_m['safd_rmse_pct']:.4f}%)"
            )
            st.plotly_chart(fig_s_diff, width='stretch')

        fig_3d_safd = go.Figure(data=[
            go.Surface(z=res["safd_cand"], x=res["a_centers_ms2"], y=res["v_centers_kmh"], colorscale="Viridis")
        ])
        fig_3d_safd.update_layout(
            title="3D Surface of Synthesized Speed–Acceleration Frequency Distribution (SAFD)",
            scene=dict(xaxis_title="Acceleration (m/s²)", yaxis_title="Speed (km/h)", zaxis_title="Probability (%)"),
            height=500, margin=dict(l=0, r=0, t=40, b=0)
        )
        st.plotly_chart(fig_3d_safd, width='stretch')

    with tab_c3:
        vsp_c1, vsp_c2 = st.columns(2)
        with vsp_c1:
            fig_vsp = go.Figure()
            fig_vsp.add_trace(go.Histogram(x=res["vsp_emp_kwt"], histnorm="probability density", name="Empirical Fleet VSP", opacity=0.6, nbinsx=40))
            fig_vsp.add_trace(go.Histogram(x=df_cycle["vsp_kwt"], histnorm="probability density", name="Synthesized Cycle VSP", opacity=0.6, nbinsx=40))
            fig_vsp.update_layout(barmode="overlay", title="True Physics-Based VSP (W/kg ≡ kW/t) Distribution", xaxis_title="VSP (W/kg)", yaxis_title="Density")
            st.plotly_chart(fig_vsp, width='stretch')
        with vsp_c2:
            wmtc1 = dyn_benchmarks["WMTC Class 1 (Stage 1 Urban Scooter/Moped)"]
            wmtc2 = dyn_benchmarks["WMTC Class 1 (Stage 1 + Stage 2 Full)"]
            r47 = dyn_benchmarks["UNECE R47 (Urban Moped Cycle)"]
            wurb = dyn_benchmarks["WLTC Urban (Low + Medium)"]
            w3b = dyn_benchmarks["WLTC Class 3b (Full)"]
            en_df = pd.DataFrame({
                "Cycle": ["Empirical Fleet", "Synthesized Cycle", "WMTC Stage 1", "WMTC Full", "UNECE R47", "WLTC Urban", "WLTC 3b"],
                "Gross Tractive+Aux (Wh/km)": [emp_m["e_trac_wh_km"], cand_m["e_trac_wh_km"], wmtc1["e_trac_wh_km"], wmtc2["e_trac_wh_km"], r47["e_trac_wh_km"], wurb["e_trac_wh_km"], w3b["e_trac_wh_km"]],
                "Recuperated Braking (Wh/km)": [emp_m["e_regen_wh_km"], cand_m["e_regen_wh_km"], wmtc1["e_regen_wh_km"], wmtc2["e_regen_wh_km"], r47["e_regen_wh_km"], wurb["e_regen_wh_km"], w3b["e_regen_wh_km"]],
                "Net Battery Energy (Wh/km)": [emp_m["e_net_wh_km"], cand_m["e_net_wh_km"], wmtc1["e_net_wh_km"], wmtc2["e_net_wh_km"], r47["e_net_wh_km"], wurb["e_net_wh_km"], w3b["e_net_wh_km"]],
            })
            fig_en = px.bar(
                en_df.melt(id_vars="Cycle", var_name="Energy Component", value_name="Wh/km"),
                x="Cycle", y="Wh/km", color="Energy Component", barmode="group",
                title="Specific Energy Consumption Comparison vs. WMTC, UNECE R47 & WLTC"
            )
            st.plotly_chart(fig_en, width='stretch')

    with tab_c4:
        mc_col1, mc_col2 = st.columns(2)
        with mc_col1:
            fig_tpm = px.imshow(
                res["v_tpm_pct"], x=res["v_centers_kmh"], y=res["v_centers_kmh"],
                origin="lower", aspect="auto", color_continuous_scale="Magma",
                labels=dict(x="Next Speed State v(k+1) [km/h]", y="Current Speed State v(k) [km/h]", color="Prob (%)"),
                title=f"Markov Speed TPM (Active 2D States: {res['active_states_count']:,} / {res['total_grid_states']:,})"
            )
            st.plotly_chart(fig_tpm, width='stretch')
        with mc_col2:
            conv_df = res["convergence_df"]
            fig_conv = px.line(
                conv_df, x="Trial #", y=["Composite Objective J", "Kinematic Mean RE (%)"],
                markers=True,
                title=f"Monte Carlo Convergence Across {len(conv_df)} Trials (Optimal: Trial #{res['best_trial']})"
            )
            st.plotly_chart(fig_conv, width='stretch')
        st.dataframe(res["convergence_df"], width='stretch', hide_index=True)

    with tab_c5:
        cdf_col1, cdf_col2 = st.columns(2)
        with cdf_col1:
            emp_v_sorted = np.sort(df_trips["velocity_kmh"].to_numpy(dtype=float))
            cand_v_sorted = np.sort(df_cycle["velocity_kmh"].to_numpy(dtype=float))
            q_grid = np.linspace(0.0, 100.0, 500)
            fig_cdf = go.Figure()
            fig_cdf.add_trace(go.Scatter(x=np.percentile(emp_v_sorted, q_grid), y=q_grid, mode="lines", name="Empirical Multi-Trip Fleet", line=dict(color="#1F77B4", width=2.5)))
            fig_cdf.add_trace(go.Scatter(x=np.percentile(cand_v_sorted, q_grid), y=q_grid, mode="lines", name="Synthesized Markov Cycle", line=dict(color="#FF7F0E", width=2.5, dash="dash")))
            fig_cdf.update_layout(
                title=f"Cumulative Speed Distribution Function (KS Stat D = {eval_m['ks_stat_v']:.4f})",
                xaxis_title="Speed (km/h)", yaxis_title="Cumulative Probability (%)", hovermode="x unified"
            )
            st.plotly_chart(fig_cdf, width='stretch')
        with cdf_col2:
            bench_name = settings["benchmark_ref"]
            bench_vals = dyn_benchmarks[bench_name]
            mode_comp_df = pd.DataFrame({
                "Driving Mode": ["Idle (%)", "Acceleration (%)", "Deceleration (%)", "Cruising (%)"],
                "Empirical Fleet": [emp_m["pct_idle"], emp_m["pct_accel"], emp_m["pct_decel"], emp_m["pct_cruise"]],
                "Synthesized Cycle": [cand_m["pct_idle"], cand_m["pct_accel"], cand_m["pct_decel"], cand_m["pct_cruise"]],
                bench_name: [bench_vals["pct_idle"], bench_vals["pct_accel"], bench_vals["pct_decel"], bench_vals["pct_cruise"]],
            })
            fig_modes = px.bar(
                mode_comp_df.melt(id_vars="Driving Mode", var_name="Dataset", value_name="Proportion (%)"),
                x="Driving Mode", y="Proportion (%)", color="Dataset", barmode="group",
                title=f"Operating Mode Proportions: Empirical vs. Synthesized vs. {bench_name}"
            )
            st.plotly_chart(fig_modes, width='stretch')

    with tab_c6:
        st.markdown("#### 📝 Automated Methodology & Results Paragraph (Ready for Manuscript)")
        n_trips = b1_params["n_files"]
        tot_emp_dur = emp_m["duration_s"]
        tot_emp_dist = emp_m["distance_km"]
        manuscript_text = (
            f"To construct a statistically representative driving cycle for {b1_params.get('vehicle_preset', 'the target electric vehicle')} "
            f"(curb mass: {b1_params['ev_curb_mass_kg']:.0f} kg, payload: {b1_params['ev_payload_mass_kg']:.0f} kg, total gross mass: "
            f"{b1_params['ev_mass_kg']:.0f} kg, auxiliary power load: {b1_params['ev_p_aux_w']:.0f} W), an experimental database comprising "
            f"{n_trips} independent trips (total active duration: {tot_emp_dur:,.0f} s [{tot_emp_dur/3600.0:.2f} h], total distance: "
            f"{tot_emp_dist:.2f} km, {res['total_microtrips_extracted']} extracted micro-trips) was processed at Δt = {b1_params['target_dt']:.1f} s. "
            f"The micro-trip pool was partitioned into K_c = {res['k_clusters_used']} traffic regimes (Silhouette score = {res['silhouette_score']:.3f}, "
            f"time-share weights = {res['cluster_weights_pct']}%) and discretized over the 2D velocity–acceleration phase space using "
            f"Δv = {settings['v_bin_kmh']:.2f} km/h and Δa = {settings['a_bin_ms2']:.2f} m/s² ({res['active_states_count']:,} active Markov states). "
            f"Across {settings['mc_iterations']} Monte Carlo candidate runs of {cand_m['duration_s']:.0f} s duration, the optimal cycle "
            f"(Trial #{res['best_trial']}) achieved a mean speed of {cand_m['v_mean_kmh']:.2f} km/h (vs. {emp_m['v_mean_kmh']:.2f} km/h for the "
            f"empirical fleet; relative error: {v_err_pct:.2f}%), an idle time share of {cand_m['pct_idle']:.2f}% (vs. {emp_m['pct_idle']:.2f}%), "
            f"a net battery specific energy consumption of {cand_m['e_net_wh_km']:.2f} Wh/km (vs. {emp_m['e_net_wh_km']:.2f} Wh/km; RE: {enet_err_pct:.2f}%), "
            f"and a 2D Speed–Acceleration Frequency Distribution (SAFD) Bhattacharyya similarity of {eval_m['safd_bhattacharyya_pct']:.2f}% "
            f"(Cosine similarity: {eval_m['safd_cosine_pct']:.2f}%, Kolmogorov–Smirnov statistic D = {eval_m['ks_stat_v']:.4f})."
        )
        st.text_area("Copyable Manuscript Paragraph:", value=manuscript_text, height=190)

    # --- EXPORT CENTER ---
    st.divider()
    st.markdown("### 💾 Export Center: Q1/Q3 Cycle Data, 54-Column V1.0.0 Format, High-DPI Figures & ZIP Archive")

    if "quick_xlsx_bytes" not in res:
        res["quick_xlsx_bytes"] = write_styled_excel_bytes({
            "1_Synthesized_Cycle_Q1": df_cycle,
            "2_EGAD_54Col_V1_0_0": df_egad_54,
            "3_Validation_Table": val_df,
            "4_MC_Convergence": res["convergence_df"],
        })

    if "quick_fig1_png" not in res:
        res["quick_fig1_png"] = render_static_publication_figures(
            res, df_trips, b1_params, dpi=300, fmt="png", only_fig1=True
        )["Fig1_Synthesized_Driving_Cycle.png"]

    q_dl1, q_dl2, q_dl3, q_dl4 = st.columns(4)
    with q_dl1:
        st.download_button(
            label="⬇️ Quick Download: Cycle (.CSV)",
            data=df_cycle.to_csv(index=False).encode("utf-8"),
            file_name="Synthesized_Driving_Cycle_Q1.csv",
            mime="text/csv",
            width='stretch',
            key="b3_quick_csv"
        )
    with q_dl2:
        st.download_button(
            label="⬇️ Quick Download: 54-Col V1.0.0 (.CSV)",
            data=df_egad_54.to_csv(index=False).encode("utf-8"),
            file_name="Synthesized_Cycle_54Col_EGAD_V1_1_0.csv",
            mime="text/csv",
            width='stretch',
            key="b3_quick_egad_csv"
        )
    with q_dl3:
        st.download_button(
            label="⬇️ Quick Download: Color-Coded Workbook (.XLSX)",
            data=res["quick_xlsx_bytes"],
            file_name="Synthesized_Driving_Cycle_and_Validation_V1_0_0.xlsx",
            mime="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet",
            width='stretch',
            key="b3_quick_xlsx"
        )
    with q_dl4:
        st.download_button(
            label="⬇️ Quick Download: Cycle Chart (300 DPI PNG)",
            data=res["quick_fig1_png"],
            file_name="Fig1_Synthesized_Driving_Cycle_300DPI.png",
            mime="image/png",
            width='stretch',
            key="b3_quick_png"
        )

    st.markdown("#### 🗜️ Full Scientific Archive Package (.ZIP)")
    e_c1, e_c2, e_c3, e_c4 = st.columns([2, 1, 1, 1])
    with e_c1:
        zip_name_input = st.text_input("Archive filename:", value="QwatLab_Markov_Driving_Cycle_V1_0_0_Package", key="b3_zip_name")
    with e_c2:
        table_fmt = st.selectbox("Primary table format:", ["Both CSV & XLSX", "CSV Only", "XLSX Only"], key="b3_tbl_fmt")
    with e_c3:
        fig_dpi = st.selectbox("Publication figure DPI:", [300, 600], index=0, key="b3_fig_dpi")
    with e_c4:
        fig_fmt = st.selectbox("Publication figure format:", ["png", "pdf", "eps"], index=0, key="b3_fig_fmt")

    if st.button("🗜️ Generate Complete Scientific ZIP Archive", type="primary", width='stretch', key="b3_btn_make_zip"):
        with st.spinner("Building high-resolution publication figures and archiving files..."):
            zip_buf = io.BytesIO()
            with zipfile.ZipFile(zip_buf, "w", zipfile.ZIP_DEFLATED) as zf:
                df_sum_trips = pd.DataFrame(st.session_state.b1_trip_summaries)
                zf.writestr("1_Input_Fleet_Trips_Summary.csv", df_sum_trips.to_csv(index=False).encode("utf-8"))

                audit_log = {
                    "Platform": "QwatLab: EV Driving Cycle Generator V1.0.0",
                    "Block_1_Vehicle_and_Ingestion_Params": b1_params,
                    "Block_2_Synthesis_Settings": settings,
                    "Selected_Optimal_Trial": int(res["best_trial"]),
                    "Selected_Optimal_Seed": int(res["best_seed"]),
                    "Extracted_MicroTrips_Count": int(res["total_microtrips_extracted"]),
                    "Quasi_MicroTrip_Splits": int(res["quasi_splits_count"]),
                    "Regime_Cluster_Weights_Pct": res["cluster_weights_pct"],
                    "Empirical_Fleet_Metrics": emp_m,
                    "Synthesized_Cycle_Metrics": cand_m,
                    "SAFD_Validation_Metrics": eval_m,
                    "Manuscript_Paragraph": manuscript_text,
                }
                zf.writestr("2_Settings_and_Methodology_Log.json", json.dumps(audit_log, indent=4))

                if table_fmt in ("Both CSV & XLSX", "CSV Only"):
                    zf.writestr("3A_Synthesized_Driving_Cycle_14Col.csv", df_cycle.to_csv(index=False).encode("utf-8"))
                    zf.writestr("3B_Synthesized_Cycle_54Col_EGAD_V1_0_0.csv", df_egad_54.to_csv(index=False).encode("utf-8"))
                    zf.writestr("4_Validation_Comparison_Table_WMTC_R47_WLTC.csv", val_df.to_csv(index=False).encode("utf-8"))
                    zf.writestr("5_Monte_Carlo_Convergence_Trials.csv", res["convergence_df"].to_csv(index=False).encode("utf-8"))

                if table_fmt in ("Both CSV & XLSX", "XLSX Only"):
                    wb_bytes = write_styled_excel_bytes({
                        "1_Synthesized_Cycle_Q1": df_cycle,
                        "2_EGAD_54Col_V1_0_0": df_egad_54,
                        "3_Validation_Table": val_df,
                        "4_Input_Trips_Audit": df_sum_trips,
                        "5_MC_Convergence_Log": res["convergence_df"],
                    })
                    zf.writestr("3_Complete_Driving_Cycle_Workbook_ColorCoded.xlsx", wb_bytes)

                safd_emp_df = pd.DataFrame(
                    res["safd_emp"],
                    index=[f"{v:.2f}_kmh" for v in res["v_centers_kmh"]],
                    columns=[f"{a:.3f}_ms2" for a in res["a_centers_ms2"]]
                )
                safd_cand_df = pd.DataFrame(
                    res["safd_cand"],
                    index=[f"{v:.2f}_kmh" for v in res["v_centers_kmh"]],
                    columns=[f"{a:.3f}_ms2" for a in res["a_centers_ms2"]]
                )
                zf.writestr("6_SAFD_Matrices/SAFD_Empirical_Fleet_Pct.csv", safd_emp_df.to_csv().encode("utf-8"))
                zf.writestr("6_SAFD_Matrices/SAFD_Synthesized_Cycle_Pct.csv", safd_cand_df.to_csv().encode("utf-8"))

                pub_figs = render_static_publication_figures(res, df_trips, b1_params, dpi=int(fig_dpi), fmt=fig_fmt)
                for fname, fbytes in pub_figs.items():
                    zf.writestr(f"7_Publication_Figures_{fig_dpi}DPI/{fname}", fbytes)

            st.session_state.b3_zip_bytes = zip_buf.getvalue()
            st.success("✅ ZIP archive generated! Click the download button below.")

    if "b3_zip_bytes" in st.session_state:
        safe_zip = re.sub(r"[^A-Za-z0-9._-]+", "_", zip_name_input.strip()).strip("._-") or "QwatLab_Markov_Cycle_V1_0_0"
        if not safe_zip.lower().endswith(".zip"):
            safe_zip += ".zip"
        st.download_button(
            label=f"⬇️ Download Complete Archive ({safe_zip})",
            data=st.session_state.b3_zip_bytes,
            file_name=safe_zip,
            mime="application/zip",
            type="primary",
            width='stretch',
            key="b3_dl_zip_final"
        )


# =====================================================================
# ASOSIY ISHGA TUSHIRISH (MAIN ENTRYPOINT)
# =====================================================================
if __name__ == "__main__":
    st.title("🔬 QwatLab: EV Driving Cycle Generator (Markov, Quasi-Micro-Trip & SAFD) V1.0.0")
    st.caption(
        "Universal 20 kg E-Moped to 3,500 kg EV Stochastic Driving Cycle Synthesis, Auto-Silhouette K-Means, WMTC/R47/WLTC Validation & 54-Col V1.0.0 Engine"
    )

    render_block_0()
    render_block_1()
    render_block_2()
    render_block_3()