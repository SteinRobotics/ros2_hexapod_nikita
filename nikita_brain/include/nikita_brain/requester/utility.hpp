/*******************************************************************************
 * Copyright (c) 2025 Christian Stein
 ******************************************************************************/

#pragma once

#include <sstream>

namespace brain {

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2) {
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return std::move(out).str();
}
}  // namespace brain