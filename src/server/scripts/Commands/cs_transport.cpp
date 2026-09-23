/*
 * This file is part of the AzerothCore project.
 *
 * Generic live-transport tracker: lists every transport (boat / zeppelin /
 * gunship / turtle / elevator-transport) currently loaded on the given map
 * (default: the session's map, or all world maps from the console) with its
 * live position, passenger count and DBC progress state.
 *
 * Usage: .transports [mapId]
 */

#include "Chat.h"
#include "ChatCommand.h"
#include "CommandScript.h"
#include "Map.h"
#include "MapMgr.h"
#include "Object.h"
#include "RBAC.h"
#include "ScriptMgr.h"
#include "StringFormat.h"
#include "Transport.h"
#include "TransportMgr.h"
#include "WorldSession.h"
#include <chrono>
#include <thread>

using namespace Acore::ChatCommands;

/*static*/ bool HandleTransportsCommand(ChatHandler* handler, char const* args)
{
    uint32 targetMap = 0;
    bool haveTarget = false;
    if (args && *args)
    {
        targetMap = (uint32)atol(args);
        haveTarget = true;
    }

    WorldSession* session = handler->GetSession();
    Player* player = session ? session->GetPlayer() : nullptr;

    // Collect maps to report on.
    std::vector<Map*> mapsToCheck;
    if (haveTarget)
    {
        Map* map = sMapMgr->FindMap(targetMap, 0);
        if (!map)
        {
            handler->PSendSysMessage("Map {} not loaded/unknown.", targetMap);
            return true;
        }
        mapsToCheck.push_back(map);
    }
    else if (player)
    {
        mapsToCheck.push_back(player->GetMap());
    }
    else
    {
        // Console: report the outdoor world maps (transports only exist in
        // persistent outdoor zones: continents + Outland/Kalimdor/EB/DB).
        static const uint32 worldMapIds[] = { 0, 1, 14, 530, 531, 532, 533, 554, 556, 571, 572, 574 };
        for (uint32 mapId : worldMapIds)
        {
            if (Map* map = sMapMgr->FindBaseMap(mapId))
                mapsToCheck.push_back(map);
        }
    }

    uint32 total = 0;
    for (Map* map : mapsToCheck)
    {
        if (!map)
            continue;
        TransportsContainer const& transports = map->GetAllTransports();
        if (transports.empty())
            continue;

        handler->PSendSysMessage("Transports on map {} ({}):", map->GetId(), transports.size());
        for (Transport* t : transports)
        {
            if (!t)
                continue;
            ++total;
            // Moving vs docked: sample the position twice (boats travel up to
            // 10y/s, so a 0.5s delta is definitive). The DBC period field is
            // the total route time, not a stop countdown — no state in it.
            float const sx = t->GetPositionX(), sy = t->GetPositionY();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            float const dx = t->GetPositionX() - sx;
            float const dy = t->GetPositionY() - sy;
            std::string state;
            if (dynamic_cast<MotionTransport*>(t))
                state = (dx * dx + dy * dy > 0.01f) ? "moving" : "docked/stationary";
            else
                state = "static";
            uint32 const progress = t->GetPathProgress();
            handler->PSendSysMessage("  entry={} '{}' guid={} map={} pos=({:.2f},{:.2f},{:.2f}) ang={:.2f} passengers={} progress={}ms [{}]",
                                     t->GetEntry(), t->GetName(), t->GetGUID().ToString(), t->GetMapId(),
                                     t->GetPositionX(), t->GetPositionY(), t->GetPositionZ(),
                                     t->GetOrientation(), t->GetPassengers().size(), progress, state);
        }
    }

    if (total == 0)
        handler->PSendSysMessage("No transports {}.", haveTarget ? "on map" : "on these maps");
    else
        handler->PSendSysMessage("Total transports listed: {}", total);
    return true;
}

class transport_commandscript : public CommandScript
{
public:
    transport_commandscript() : CommandScript("transport_commandscript") { }

    ChatCommandTable GetCommands() const override
    {
        static ChatCommandTable transportCommandTable =
        {
            { "transports",    HandleTransportsCommand,      rbac::RBAC_PERM_COMMAND_DEBUG_INFO, Console::Yes }
        };
        return transportCommandTable;
    }
};

void AddSC_transport_commandscript()
{
    new transport_commandscript();
}
