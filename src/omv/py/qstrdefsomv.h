/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * qstrs specific to openmv
 *
 */

// Image module
Q(image)
Q(Image)
Q(rgb_to_lab)
Q(lab_to_rgb)
Q(rgb_to_grayscale)
Q(grayscale_to_rgb)
Q(HaarCascade)
Q(search)
Q(SEARCH_EX)
Q(SEARCH_DS)
Q(EDGE_CANNY)
Q(EDGE_SIMPLE)
Q(CORNER_FAST)
Q(CORNER_AGAST)
Q(load_descriptor)
Q(save_descriptor)
Q(match_descriptor)

// Image class
Q(copy)
Q(copy_to_fb)
Q(save)
Q(compress)
Q(compress_for_ide)
Q(compressed)
Q(compressed_for_ide)
Q(width)
Q(height)
Q(format)
Q(size)
Q(get_pixel)
Q(set_pixel)
Q(draw_line)
Q(draw_rectangle)
Q(draw_circle)
Q(draw_string)
Q(draw_cross)
Q(draw_keypoints)
Q(binary)
Q(invert)
Q(and)
Q(nand)
Q(or)
Q(nor)
Q(xor)
Q(xnor)
Q(erode)
Q(dilate)
Q(negate)
Q(difference)
Q(replace)
Q(blend)
Q(morph)
Q(midpoint)
Q(mean)
Q(mode)
Q(median)
Q(gaussian)
Q(midpoint_pool)
Q(midpoint_pooled)
Q(mean_pool)
Q(mean_pooled)
Q(find_template)
Q(find_displacement)
Q(kp_desc)
Q(lbp_desc)
Q(Cascade)
Q(histeq)
Q(mask_ellipse)
Q(find_features)
Q(find_keypoints)
Q(find_lbp)
Q(find_eye)
Q(find_lines)
Q(find_edges)
Q(find_hog)
Q(cmp_lbp)
Q(quality)
Q(color)
Q(roi)
Q(offset)
Q(threshold)
Q(mul)
Q(add)
Q(bias)
Q(percentile)
Q(normalized)
Q(filter_outliers)
Q(scale_factor)
Q(max_keypoints)
Q(corner_detector)

// Lcd Module
Q(lcd)
Q(type)
Q(has_blctl)
Q(rotation)
Q(set_backlight)
Q(get_backlight)
Q(display)
Q(clear)

// FIR Module
Q(fir)
Q(read_ta)
Q(read_ir)
Q(draw_ta)
Q(draw_ir)
Q(alpha)
Q(scale)
Q(refresh)
Q(resolution)

// Gif module
Q(gif)
Q(Gif)
Q(open)
Q(add_frame)
Q(loop)

// Mjpeg module
Q(mjpeg)
Q(Mjpeg)

// Led Module
Q(led)
Q(RED)
Q(GREEN)
Q(BLUE)
Q(IR)
Q(on)
Q(off)
Q(toggle)

// Time Module
Q(time)
Q(ticks)
Q(sleep)
Q(clock)
Q(Clock)

// Clock
Q(tick)
Q(fps)
Q(avg)

//Sensor Module
Q(sensor)
Q(BAYER)
Q(RGB565)
Q(YUV422)
Q(GRAYSCALE)
Q(JPEG)
Q(B40x30)
Q(B64x32)
Q(B64x64)
Q(QQCIF)
Q(QQVGA)
Q(QQVGA2)
Q(QCIF)
Q(HQVGA)
Q(QVGA)
Q(CIF)
Q(VGA)
Q(SVGA)
Q(SXGA)
Q(UXGA)
Q(OV9650)
Q(OV2640)
Q(OV7725)
Q(line_filter)
Q(value)

//SDE
Q(NORMAL)
Q(NEGATIVE)

Q(reset)
Q(snapshot)
Q(skip_frames)
Q(get_fb)
Q(get_id)
Q(set_pixformat)
Q(set_framerate)
Q(set_framesize)
Q(set_binning)
Q(set_windowing)
Q(set_gainceiling)
Q(set_contrast)
Q(set_brightness)
Q(set_saturation)
Q(set_quality)
Q(set_colorbar)
Q(set_auto_gain)
Q(set_auto_exposure)
Q(set_auto_whitebal)
Q(set_hmirror)
Q(set_vflip)
Q(set_special_effect)
Q(set_lens_correction)
Q(__write_reg)
Q(__read_reg)

// GPIOS
Q(P1)
Q(P2)
Q(P3)
Q(P4)
Q(P5)
Q(P6)
Q(PA1)
Q(PA2)
Q(PA3)
Q(PA4)
Q(PA5)
Q(PA6)
Q(PA7)
Q(PA8)
Q(PB1)
Q(PB2)
Q(PB3)
Q(PB4)
Q(IN)
Q(OUT)
Q(gpio)
Q(GPIO)
Q(low)
Q(high)

// SPI
Q(spi)
Q(read)
Q(write)
Q(write_image)

// UART
Q(uart)

// File
Q(file)
Q(close)

//Wlan
Q(wlan)
Q(WEP)
Q(WPA)
Q(WPA2)
Q(init)
Q(connect)
Q(connected)
Q(ifconfig)
Q(patch_version)
Q(patch_program)
Q(socket)
Q(send)
Q(recv)
Q(bind)
Q(listen)
Q(accept)
Q(settimeout)
Q(setblocking)
Q(select)
Q(AF_INET)
Q(AF_INET6)
Q(SOCK_STREAM)
Q(SOCK_DGRAM)
Q(SOCK_RAW)
Q(IPPROTO_IP)
Q(IPPROTO_ICMP)
Q(IPPROTO_IPV4)
Q(IPPROTO_TCP)
Q(IPPROTO_UDP)
Q(IPPROTO_IPV6)
Q(IPPROTO_RAW)

// for WINC1500 module
Q(WINC)
Q(connect)
Q(start_ap)
Q(disconnect)
Q(isconnected)
Q(connected_sta)
Q(wait_for_sta)
Q(ifconfig)
Q(fw_version)
Q(fw_dump)
Q(fw_update)
Q(scan)
Q(rssi)
Q(OPEN)
Q(WEP)
Q(WPA_PSK)
Q(802_1X)
Q(MODE_STA)
Q(MODE_AP)
Q(MODE_P2P)
Q(MODE_FIRMWARE)
Q(ssid)
Q(key)
Q(security)
Q(bssid)

// cpufreq Module
Q(cpufreq)
Q(CPUFREQ_120MHZ)
Q(CPUFREQ_144MHZ)
Q(CPUFREQ_168MHZ)
Q(CPUFREQ_192MHZ)
Q(CPUFREQ_216MHZ)
Q(get_frequency)
Q(set_frequency)

// Lens Correction
Q(lens_corr)
Q(strength)
Q(zoom)

// Get Histogram
Q(get_hist)
Q(get_histogram)
// Histogram Object
Q(histogram)
Q(bins)
Q(l_bins)
Q(a_bins)
Q(b_bins)
Q(get_percentile)
Q(get_stats)
Q(get_statistics)
Q(statistics)
// Percentile Object
// duplicate Q(percentile)
Q(value)
Q(l_value)
Q(a_value)
Q(b_value)

// Get Statistics
// duplicate Q(get_stats)
// duplicate Q(get_statistics)
// Statistics Object
// duplicate Q(statistics)
// duplicate Q(mean)
// duplicate Q(median)
// duplicate Q(mode)
Q(stdev)
// duplicate Q(min)
// duplicate Q(max)
Q(lq)
Q(uq)
Q(l_mean)
Q(l_median)
Q(l_mode)
Q(l_stdev)
Q(l_min)
Q(l_max)
Q(l_lq)
Q(l_uq)
Q(a_mean)
Q(a_median)
Q(a_mode)
Q(a_stdev)
Q(a_min)
Q(a_max)
Q(a_lq)
Q(a_uq)
Q(b_mean)
Q(b_median)
Q(b_mode)
Q(b_stdev)
Q(b_min)
Q(b_max)
Q(b_lq)
Q(b_uq)

// Find Blobs
Q(find_blobs)
Q(x_stride)
Q(y_stride)
Q(area_threshold)
Q(pixels_threshold)
Q(merge)
Q(margin)
Q(threshold_cb)
Q(merge_cb)
// duplicate Q(roi)
// Blob Object
Q(blob)
Q(rect)
Q(x)
Q(y)
Q(w)
Q(h)
Q(pixels)
Q(cx)
Q(cy)
Q(rotation)
Q(code)
Q(count)
Q(area)
Q(density)

// Find QRCodes
Q(find_qrcodes)
// duplicate Q(roi)
// QRCode Object
Q(qrcode)
// duplicate Q(rect)
// duplicate Q(x)
// duplicate Q(y)
// duplicate Q(w)
// duplicate Q(h)
Q(payload)
Q(version)
Q(ecc_level)
Q(mask)
Q(data_type)
Q(eci)
Q(is_numeric)
Q(is_alphanumeric)
Q(is_binary)
Q(is_kanji)

// Find AprilTags
Q(find_apriltags)
// duplicate Q(roi)
Q(families)
Q(fx)
Q(fy)
// duplicate Q(cx)
// duplicate Q(cy)
// AprilTag Object
Q(apriltag)
// duplicate Q(rect)
// duplicate Q(x)
// duplicate Q(y)
// duplicate Q(w)
// duplicate Q(h)
Q(id)
Q(family)
Q(hamming)
// duplicate Q(cx)
// duplicate Q(cy)
// duplicate Q(rotation)
Q(goodness)
Q(decision_margin)
Q(x_translation)
Q(y_translation)
Q(z_translation)
Q(x_rotation)
Q(y_rotation)
Q(z_rotation)
Q(TAG16H5)
Q(TAG25H7)
Q(TAG25H9)
Q(TAG36H10)
Q(TAG36H11)
Q(ARTOOLKIT)

// Find BarCodes
Q(find_barcodes)
// duplicate Q(roi)
// BarCode Object
Q(barcode)
// duplicate Q(rect)
// duplicate Q(x)
// duplicate Q(y)
// duplicate Q(w)
// duplicate Q(h)
// duplicate Q(payload)
// duplicate Q(type)
// duplicate Q(rotation)
// duplicate Q(quality)
Q(EAN2)
Q(EAN5)
Q(EAN8)
Q(UPCE)
Q(ISBN10)
Q(UPCA)
Q(EAN13)
Q(ISBN13)
Q(I25)
Q(DATABAR)
Q(DATABAR_EXP)
Q(CODABAR)
Q(CODE39)
Q(PDF417)
Q(CODE93)
Q(CODE128)

/* yuanjun */
// MPU6050
Q(mpu6050)
Q(start_chan)
Q(accel_read)
Q(gyro_read)
Q(read_mpureg)
Q(write_mpureg)
