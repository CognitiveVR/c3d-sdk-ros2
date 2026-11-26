#pragma once

#include <string>

namespace cognitive3d_ros {

// Encodings the camera path can decode to BGR. Anything else is refused with a
// once-per-topic warning rather than guessed at.
bool AcceptedImageEncoding(const std::string& encoding);

// A depth image is metres or millimetres per pixel, not colour. Encoded as video
// it produces a grey rectangle that looks like a picture and means nothing, so it
// is named separately from "unsupported": the operator has configured the wrong
// topic, and the warning should say so.
bool IsDepthEncoding(const std::string& encoding);

}  // namespace cognitive3d_ros
