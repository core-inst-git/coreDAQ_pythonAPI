# coreDAQ Python API — Programmer’s Manual

This document is the **official programmer’s manual** for the `coredaq_py_api` Python driver.  
It explains **all public functionality**, starting from a quick “getting started” guide and moving on to a detailed API reference with practical examples.

The API is designed for **photonic and optoelectronic measurements**, with emphasis on:
- calibrated optical power readout
- low-noise operation via oversampling
- gain control and autogain (LINEAR front end)
- single-shot snapshots and timer-based acquisitions
- optional external triggered start
- environmental monitoring (temperature & humidity)

---

## Table of Contents
- [Getting started](#getting-started)
- [Core concepts](#core-concepts)
  - [Front-end types](#front-end-types)
  - [Oversampling and sampling frequency](#oversampling-and-sampling-frequency)
  - [Gain stages (LINEAR)](#gain-stages-linear)
  - [Zeroing (LINEAR)](#zeroing-linear)
  - [LOG deadband (LOG)](#log-deadband-log)
- [API reference](#api-reference)
  - [Lifecycle](#lifecycle)
  - [Identity and device type](#identity-and-device-type)
  - [Port discovery](#port-discovery)
  - [Configuration](#configuration)
  - [Gain control](#gain-control)
  - [Zeroing](#zeroing)
  - [Single-shot measurements (snapshots)](#single-shot-measurements-snapshots)
  - [Acquisition control](#acquisition-control)
  - [Bulk data transfer](#bulk-data-transfer)
  - [Environmental sensors](#environmental-sensors)
  - [Utility conversions](#utility-conversions)
- [Examples](#examples)
  - [Single measurement](#single-measurement)
  - [Timer-based acquisition (free-running)](#timer-based-acquisition-free-running)
  - [Timer-based acquisition (external trigger)](#timer-based-acquisition-external-trigger)

---

## Getting started

### Requirements
```bash

pip install pyserial pyqt5 numpy matplotlib


## Core Concepts

This section explains the key concepts behind the coreDAQ system and Python API.  
Understanding these ideas will help you configure measurements correctly and interpret results with confidence.

---

### Front-end Types

coreDAQ devices are available with two fundamentally different analog front ends.  
The front-end type is detected automatically at connection time and determines which features are available.

#### LINEAR Front End
- Photodiode readout via a **linear transimpedance amplifier (TIA)**
- Discrete, selectable gain stages
- Lowest noise and best absolute accuracy
- Ideal for precision measurements, low-noise experiments, and calibration work

Features:
- Gain switching (8 gain stages)
- Factory zero correction
- Software (soft) zeroing
- Optional autogain during measurements

Typical use cases:
- Low-noise optical power monitoring
- Ring-resonator sweeps
- Extinction ratio measurements
- Detector characterization

#### LOG Front End
- Logarithmic amplifier with LUT-based calibration
- Extremely large dynamic range
- No gain switching
- Offset handled via a configurable deadband

Features:
- No gain control
- No zero subtraction
- Voltage-to-power conversion via LUT
- Deadband to suppress low-level offset drift

Typical use cases:
- Very wide dynamic range measurements
- Power monitoring across many orders of magnitude
- Situations where absolute low-noise performance is less critical

You can query the front-end type at runtime:
```python
daq.frontend_type()   # "LINEAR" or "LOG"
