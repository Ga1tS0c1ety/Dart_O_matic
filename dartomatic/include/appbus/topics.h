// dartomatic/include/appbus/topics.h

#pragma once

/*
 * Topics minimaux pour la V1 end-to-end :
 * - cmd/* : commandes (UI -> services)
 * - evt/* : événements (services -> UI, etc.)
 */

#define TOPIC_CMD_GAME_START        "cmd/game/start"
#define TOPIC_EVT_IMPACT_TRIANG     "evt/impact/triangulated"
#define TOPIC_EVT_HIT_SCORED        "evt/hit/scored"
#define TOPIC_EVT_GAME_STATE        "evt/game/state"
#define TOPIC_CMD_BOARD_CLEAR_CONF "cmd/board/clear_confirmed"