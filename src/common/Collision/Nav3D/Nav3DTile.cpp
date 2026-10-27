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

#include "Nav3DTile.h"

#include <cstring>
#include <fstream>
#include <limits>

#include <zlib.h>

namespace
{
    constexpr uint32 kMaxTilePayload = 1024u * 1024u * 1024u;

    template <typename T>
    void AppendPod(std::vector<uint8>& out, T const& value)
    {
        size_t const offset = out.size();
        out.resize(offset + sizeof(T));
        std::memcpy(out.data() + offset, &value, sizeof(T));
    }

    template <typename T>
    bool ReadPod(std::vector<uint8> const& in, size_t& offset, T& value)
    {
        if (offset + sizeof(T) > in.size())
            return false;
        std::memcpy(&value, in.data() + offset, sizeof(T));
        offset += sizeof(T);
        return true;
    }
}

bool Nav3DTile::Save(std::string const& path) const
{
    if (path.empty() || _header.CellSize <= 0.0f || _header.SizeX == 0 || _header.SizeY == 0 ||
        _header.SizeZ == 0)
        return false;

    uint64 const expectedColumns = uint64(_header.SizeX) * uint64(_header.SizeY);
    if (expectedColumns == 0 || expectedColumns > std::numeric_limits<uint32>::max() ||
        _columns.size() != size_t(expectedColumns))
        return false;
    if (_intervals.size() > std::numeric_limits<uint32>::max())
        return false;

    for (Nav3DTileColumn const& column : _columns)
    {
        if (uint64(column.IntervalOffset) + column.IntervalCount > _intervals.size())
            return false;
    }

    std::vector<uint8> payload;
    payload.reserve(_columns.size() * 8 + _intervals.size() * 6 + 8);
    AppendPod(payload, uint32(_columns.size()));
    for (Nav3DTileColumn const& column : _columns)
    {
        AppendPod(payload, column.IntervalOffset);
        AppendPod(payload, column.IntervalCount);
        AppendPod(payload, column.Reserved);
    }

    AppendPod(payload, uint32(_intervals.size()));
    for (Nav3DTileInterval const& interval : _intervals)
    {
        AppendPod(payload, interval.ZMin);
        AppendPod(payload, interval.ZMax);
        AppendPod(payload, interval.Clearance);
        AppendPod(payload, interval.Flags);
    }

    if (payload.size() > kMaxTilePayload)
        return false;

    Nav3DTileHeader header = _header;
    header.Magic = Nav3D::TileMagic;
    header.Version = Nav3D::TileVersion;
    header.HeaderSize = Nav3D::TileHeaderSize;
    header.Flags |= Nav3D::FlagCompressed | Nav3D::FlagDenseColumns;
    header.UncompressedSize = uint32(payload.size());

    uLongf compressedSize = compressBound(uLong(payload.size()));
    std::vector<uint8> compressed(compressedSize);
    if (compress2(compressed.data(), &compressedSize, payload.data(), uLong(payload.size()),
                  Z_DEFAULT_COMPRESSION) != Z_OK)
        return false;

    compressed.resize(compressedSize);
    header.CompressedSize = uint32(compressed.size());
    header.Checksum = uint32(crc32(0, payload.data(), uInt(payload.size())));

    std::ofstream out(path, std::ios::binary);
    if (!out)
        return false;

    auto const write = [&out](auto const& value)
    {
        out.write(reinterpret_cast<char const*>(&value), sizeof(value));
    };

    write(header.Magic);
    write(header.Version);
    write(header.HeaderSize);
    write(header.MapId);
    write(header.TileX);
    write(header.TileY);
    write(header.Medium);
    write(header.Flags);
    write(header.CellSize);
    write(header.OriginX);
    write(header.OriginY);
    write(header.OriginZ);
    write(header.ExtentX);
    write(header.ExtentY);
    write(header.ExtentZ);
    write(header.SizeX);
    write(header.SizeY);
    write(header.SizeZ);
    write(header.GeneratorVersion);
    write(header.GeometryHash);
    write(header.UncompressedSize);
    write(header.CompressedSize);
    write(header.Checksum);

    out.write(reinterpret_cast<char const*>(compressed.data()), std::streamsize(compressed.size()));
    return out.good();
}

bool Nav3DTile::Load(std::string const& path)
{
    _columns.clear();
    _intervals.clear();

    std::ifstream in(path, std::ios::binary);
    if (!in)
        return false;

    Nav3DTileHeader header;
    auto const read = [&in](auto& value)
    {
        in.read(reinterpret_cast<char*>(&value), sizeof(value));
        return in.good();
    };

    if (!read(header.Magic) || !read(header.Version) || !read(header.HeaderSize) ||
        !read(header.MapId) || !read(header.TileX) || !read(header.TileY) || !read(header.Medium) ||
        !read(header.Flags) || !read(header.CellSize) || !read(header.OriginX) ||
        !read(header.OriginY) || !read(header.OriginZ) || !read(header.ExtentX) ||
        !read(header.ExtentY) || !read(header.ExtentZ) || !read(header.SizeX) ||
        !read(header.SizeY) || !read(header.SizeZ) || !read(header.GeneratorVersion) ||
        !read(header.GeometryHash) || !read(header.UncompressedSize) ||
        !read(header.CompressedSize) || !read(header.Checksum))
        return false;

    if (header.Magic != Nav3D::TileMagic || header.Version != Nav3D::TileVersion ||
        header.HeaderSize != Nav3D::TileHeaderSize || header.CellSize <= 0.0f ||
        header.SizeX == 0 || header.SizeY == 0 || header.SizeZ == 0 ||
        header.UncompressedSize == 0 || header.UncompressedSize > kMaxTilePayload ||
        header.CompressedSize == 0 || header.CompressedSize > kMaxTilePayload ||
        !(header.Flags & Nav3D::FlagCompressed) || !(header.Flags & Nav3D::FlagDenseColumns))
        return false;

    std::vector<uint8> compressed(header.CompressedSize);
    in.read(reinterpret_cast<char*>(compressed.data()), std::streamsize(compressed.size()));
    if (!in.good())
        return false;

    std::vector<uint8> payload(header.UncompressedSize);
    uLongf payloadSize = header.UncompressedSize;
    if (uncompress(payload.data(), &payloadSize, compressed.data(), uLong(compressed.size())) != Z_OK ||
        payloadSize != header.UncompressedSize)
        return false;
    if (uint32(crc32(0, payload.data(), uInt(payload.size()))) != header.Checksum)
        return false;

    size_t offset = 0;
    uint32 columnCount = 0;
    if (!ReadPod(payload, offset, columnCount))
        return false;

    uint64 const expectedColumns = uint64(header.SizeX) * uint64(header.SizeY);
    if (expectedColumns == 0 || expectedColumns > std::numeric_limits<uint32>::max() ||
        columnCount != uint32(expectedColumns))
        return false;

    _columns.resize(columnCount);
    for (Nav3DTileColumn& column : _columns)
    {
        if (!ReadPod(payload, offset, column.IntervalOffset) ||
            !ReadPod(payload, offset, column.IntervalCount) ||
            !ReadPod(payload, offset, column.Reserved))
            return false;
    }

    uint32 intervalCount = 0;
    if (!ReadPod(payload, offset, intervalCount))
        return false;
    if (intervalCount > kMaxTilePayload / sizeof(Nav3DTileInterval))
        return false;

    _intervals.resize(intervalCount);
    for (Nav3DTileInterval& interval : _intervals)
    {
        if (!ReadPod(payload, offset, interval.ZMin) || !ReadPod(payload, offset, interval.ZMax) ||
            !ReadPod(payload, offset, interval.Clearance) || !ReadPod(payload, offset, interval.Flags))
            return false;
        if (interval.ZMin > interval.ZMax || interval.ZMin < 0 ||
            uint32(interval.ZMax) >= header.SizeZ)
            return false;
    }

    if (offset != payload.size())
        return false;

    for (Nav3DTileColumn const& column : _columns)
    {
        if (uint64(column.IntervalOffset) + column.IntervalCount > _intervals.size())
            return false;
    }

    _header = header;
    return true;
}

bool Nav3DTile::CellIndex(float x, float y, float z, uint32& column, int32& zCell) const
{
    if (_header.CellSize <= 0.0f || _columns.empty())
        return false;

    float const fx = (x - _header.OriginX) / _header.CellSize;
    float const fy = (y - _header.OriginY) / _header.CellSize;
    float const fz = (z - _header.OriginZ) / _header.CellSize;
    if (fx < 0.0f || fy < 0.0f || fz < 0.0f || fx >= float(_header.SizeX) ||
        fy >= float(_header.SizeY) || fz >= float(_header.SizeZ))
        return false;

    uint32 const ix = uint32(fx);
    uint32 const iy = uint32(fy);
    int32 const iz = int32(fz);
    if (ix >= _header.SizeX || iy >= _header.SizeY || iz < 0 || uint32(iz) >= _header.SizeZ)
        return false;

    uint64 const columnIndex = uint64(iy) * _header.SizeX + ix;
    if (columnIndex >= _columns.size())
        return false;

    column = uint32(columnIndex);
    zCell = iz;
    return true;
}

bool Nav3DTile::IsNavigable(float x, float y, float z, float radius) const
{
    uint32 column = 0;
    int32 zCell = 0;
    if (!CellIndex(x, y, z, column, zCell))
        return false;

    Nav3DTileColumn const& cellColumn = _columns[column];
    if (cellColumn.IntervalCount == 0 ||
        uint64(cellColumn.IntervalOffset) + cellColumn.IntervalCount > _intervals.size())
        return false;

    size_t low = 0;
    size_t high = cellColumn.IntervalCount;
    while (low < high)
    {
        size_t const mid = (low + high) / 2;
        Nav3DTileInterval const& interval = _intervals[cellColumn.IntervalOffset + mid];
        if (interval.ZMin <= zCell)
            low = mid + 1;
        else
            high = mid;
    }

    if (low == 0)
        return false;

    Nav3DTileInterval const& interval = _intervals[cellColumn.IntervalOffset + low - 1];
    if (zCell < interval.ZMin || zCell > interval.ZMax)
        return false;

    return float(interval.Clearance) * Nav3D::ClearanceUnit >= radius;
}

uint8 Nav3DTile::GetClearance(float x, float y, float z) const
{
    uint32 column = 0;
    int32 zCell = 0;
    if (!CellIndex(x, y, z, column, zCell))
        return 0;

    Nav3DTileColumn const& cellColumn = _columns[column];
    if (cellColumn.IntervalCount == 0 ||
        uint64(cellColumn.IntervalOffset) + cellColumn.IntervalCount > _intervals.size())
        return 0;

    size_t low = 0;
    size_t high = cellColumn.IntervalCount;
    while (low < high)
    {
        size_t const mid = (low + high) / 2;
        Nav3DTileInterval const& interval = _intervals[cellColumn.IntervalOffset + mid];
        if (interval.ZMin <= zCell)
            low = mid + 1;
        else
            high = mid;
    }

    if (low == 0)
        return 0;

    Nav3DTileInterval const& interval = _intervals[cellColumn.IntervalOffset + low - 1];
    if (zCell < interval.ZMin || zCell > interval.ZMax)
        return 0;

    return interval.Clearance;
}
