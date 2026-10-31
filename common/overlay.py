"""Readable on-screen text: coloured text with a black outline, visible on bright AND dark backgrounds."""

import cv2


def draw_text(image, text, origin, colour=(255, 255, 255), scale=0.65, thickness=2):
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thickness, cv2.LINE_AA)
