/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Plane.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
  constructor for main Plane class
 */
bool Plane::is_last_waypoint_before_land() {
    // Проверка, является ли текущая точка последней перед LAND
    return mission.is_last_before_land();
}

Vector3f Plane::get_current_position() const {
    return Vector3f(current_loc.lat * 1.0e-7f, // Перевод широты в float
                    current_loc.lng * 1.0e-7f, // Перевод долготы в float
                    current_loc.alt * 1.0f);   // Высота в метрах
}

Plane::Plane(void)
{
    // C++11 doesn't allow in-class initialisation of bitfields
    auto_state.takeoff_complete = true;
}

Plane plane;
AP_Vehicle& vehicle = plane;
