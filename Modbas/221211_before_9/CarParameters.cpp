/**
  * Mini-Auto-Drive MAD
  *
  * Copyright (C) 2020 Frank Traenkle
  * http://www.modbas.de
  *
  * Car Parameters
  *
  * This file is part of Mini-Auto-Drive.
  *
  * Mini-Auto-Drive is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * Mini-Auto-Drive is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.
  *
  */

#include "CarParameters.h"

CarParameters* CarParameters::singleton = nullptr;

static CarParameters dummy;

CarParameters::CarParameters() noexcept
{
  singleton = this;
}
