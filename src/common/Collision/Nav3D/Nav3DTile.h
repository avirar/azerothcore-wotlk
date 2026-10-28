/*
 * This file is part of the AzerothCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If not,
 * see <http://www.gnu.org/licenses/>.
 */

#ifndef PLAYERBOTS_NAV3DTILE_H
#define PLAYERBOTS_NAV3DTILE_H

#include "Define.h"
#include <string>
#include <vector>

namespace Nav3D
{
    constexpr uint32 TileMagic = 0x3154334E;  // "N3T1" little-endian
    constexpr uint16 TileVersion = 1;
    constexpr uint16 TileHeaderSize = 82;
    constexpr uint8 MediumWater = 1;
    constexpr uint8 MediumAir = 2;
    constexpr uint8 MediumAirDetail = 3;  // local 1-yd WMO/cave detail tile
    constexpr uint8 FlagCompressed = 0x01;
    constexpr uint8 FlagDenseColumns = 0x02;
    constexpr uint8 FlagHasSolid = 0x04;
    constexpr uint8 FlagLiveLos = 0x08;
    constexpr float ClearanceUnit = 0.25f;
}

struct Nav3DTileHeader
{
    uint32 Magic = Nav3D::TileMagic;
    uint16 Version = Nav3D::TileVersion;
    uint16 HeaderSize = Nav3D::TileHeaderSize;
    uint32 MapId = 0;
    uint16 TileX = 0;
    uint16 TileY = 0;
    uint8 Medium = Nav3D::MediumWater;
    uint8 Flags = Nav3D::FlagCompressed | Nav3D::FlagDenseColumns;
    float CellSize = 1.0f;
    float OriginX = 0.0f;
    float OriginY = 0.0f;
    float OriginZ = 0.0f;
    float ExtentX = 0.0f;
    float ExtentY = 0.0f;
    float ExtentZ = 0.0f;
    uint32 SizeX = 0;
    uint32 SizeY = 0;
    uint32 SizeZ = 0;
    uint32 GeneratorVersion = 1;
    uint64 GeometryHash = 0;
    uint32 UncompressedSize = 0;
    uint32 CompressedSize = 0;
    uint32 Checksum = 0;
};

struct Nav3DTileInterval
{
    int16 ZMin = 0;
    int16 ZMax = 0;
    uint8 Clearance = 0;
    uint8 Flags = 0;
};

struct Nav3DTileColumn
{
    uint32 IntervalOffset = 0;
    uint16 IntervalCount = 0;
    uint16 Reserved = 0;
};

// Version 1 stores a dense column index and per-column Z intervals. Sparse blocks and
// per-cell detail clearance are reserved for a later format version.
class Nav3DTile
{
public:
    bool Load(std::string const& path);
    bool Save(std::string const& path) const;

    [[nodiscard]] bool IsNavigable(float x, float y, float z, float radius = 0.5f) const;
    [[nodiscard]] uint8 GetClearance(float x, float y, float z) const;

    [[nodiscard]] Nav3DTileHeader const& Header() const { return _header; }
    Nav3DTileHeader& Header() { return _header; }
    [[nodiscard]] std::vector<Nav3DTileColumn> const& Columns() const { return _columns; }
    std::vector<Nav3DTileColumn>& Columns() { return _columns; }
    [[nodiscard]] std::vector<Nav3DTileInterval> const& Intervals() const { return _intervals; }
    std::vector<Nav3DTileInterval>& Intervals() { return _intervals; }

private:
    [[nodiscard]] bool CellIndex(float x, float y, float z, uint32& column, int32& zCell) const;

    Nav3DTileHeader _header;
    std::vector<Nav3DTileColumn> _columns;
    std::vector<Nav3DTileInterval> _intervals;
};

#endif
