#pragma once

#include <cstdint>
#include <string>

namespace cognitive3d_ros {

// Width and height read out of a JPEG's SOF marker, without decoding it.
//
// This exists for the pass-through path: a `CompressedImage` that is already JPEG
// and needs no resize is forwarded byte for byte, and its dimensions are the one
// thing the wire needs that the ROS message does not carry. Decoding a frame to
// learn two numbers we are not going to use would cost more than the encode we
// just avoided.
//
// Every SOF marker counts, not only SOF0: a progressive JPEG is SOF2 and is what
// most encoders produce when asked for one, and reading only baseline would send
// 0x0 for it. The markers that are NOT frame headers and happen to fall in the
// same range are excluded by name -- DHT (C4), JPG (C8) and DAC (CC).
bool JpegDimensions(const std::string& jpeg, std::uint16_t* width, std::uint16_t* height);

// Encodings the camera path can decode to BGR. Anything else is refused with a
// once-per-topic warning rather than guessed at.
bool AcceptedImageEncoding(const std::string& encoding);

// A depth image is metres or millimetres per pixel, not colour. Run through a
// JPEG encoder it produces a grey rectangle that looks like a picture and means
// nothing, so it is named separately from "unsupported": the operator has
// configured the wrong topic, and the warning should say so.
bool IsDepthEncoding(const std::string& encoding);

// Whether a `CompressedImage`'s `format` field names JPEG. The field is free text
// in practice -- "jpeg", "bgr8; jpeg compressed bgr8" and "rgb8; jpeg compressed
// bgr8" are all published by the ROS image transport -- so this is a search, not
// an equality test.
bool CompressedFormatIsJpeg(const std::string& format);

}  // namespace cognitive3d_ros
