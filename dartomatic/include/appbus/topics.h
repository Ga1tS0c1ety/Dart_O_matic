#pragma once

/*
 * Topics AppBus
 * -------------
 * cmd/* : commandes envoyées par UI / arbitrage
 * evt/* : événements publiés par services / RT
 */

#define TOPIC_CMD_GAME_START          "cmd/game/start"
#define TOPIC_CMD_GAME_UNDO           "cmd/game/undo"
#define TOPIC_CMD_GAME_OVERRIDE_LAST  "cmd/game/override_last"
#define TOPIC_CMD_GAME_ADD_MANUAL_HIT "cmd/game/add_manual_hit"
#define TOPIC_CMD_BOARD_CLEAR_CONF    "cmd/board/clear_confirmed"

#define TOPIC_EVT_IMPACT_TRIANG       "evt/impact/triangulated"
#define TOPIC_EVT_HIT_SCORED          "evt/hit/scored"
#define TOPIC_EVT_GAME_STATE          "evt/game/state"