#include "colormap.h"
#include <math.h> // isnan()

constexpr uint8_t Colormap::m_reds[];
constexpr uint8_t Colormap::m_greens[];
constexpr uint8_t Colormap::m_blues[];

rgb_t Colormap::color(const float value) const
{
   rgb_t color = {0,0,0};
   if (isnan(value))
   {
      return {0,0,0};
   }
   if (value < m_min_val)
   {
      color.red = m_reds[0];
      color.green = m_greens[0];
      color.blue = m_blues[0];
   }
   else if (value >= m_max_val)
   {
      color.red = m_reds[m_no_of_colors - 1];
      color.green = m_greens[m_no_of_colors - 1];
      color.blue = m_blues[m_no_of_colors - 1];
   }
   else
   {
      // Linearly interpolate between colors.
      float normalized_value =
         (value - m_min_val) / (m_max_val - m_min_val) * (static_cast<float>(m_no_of_colors) - 1.);
      unsigned int segment = static_cast<unsigned int>(normalized_value);
      if (segment >= m_no_of_colors) // This should not happen.
      {
         return {0,0,0};
      }
      float factor = normalized_value - static_cast<float>(segment);
      float interp = factor * static_cast<float>(m_reds[segment + 1] - m_reds[segment]);
      if (interp < 0)
         color.red = m_reds[segment] - static_cast<uint8_t>(-interp);
      else
         color.red = m_reds[segment] + static_cast<uint8_t>(interp);
      interp = factor * static_cast<float>(m_greens[segment + 1] - m_greens[segment]);
      if (interp < 0)
         color.green = m_greens[segment] - static_cast<uint8_t>(-interp);
      else
         color.green = m_greens[segment] + static_cast<uint8_t>(interp);
      interp = factor * static_cast<float>(m_blues[segment + 1] - m_blues[segment]);
      if (interp < 0)
         color.blue = m_blues[segment] - static_cast<uint8_t>(-interp);
      else
         color.blue = m_blues[segment] + static_cast<uint8_t>(interp);
   }
   return color;
}
