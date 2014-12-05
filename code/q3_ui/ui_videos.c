/*
=======================================================================

VIDEOS MENU

=======================================================================
*/

#include "ui_local.h"

#define VIDEOS_MENU_VERTICAL_SPACING		34

#define ART_BACK0				"menu/art/back_0"
#define ART_BACK1				"menu/art/back_1"	
#define ART_FRAMEL				"menu/art/frame2_l"
#define ART_FRAMER				"menu/art/frame1_r"

#define ID_DEMOS				10
#define ID_CINEMATICS			11
#define ID_BACK					12

typedef struct {
	menuframework_s	menu;

	menutext_s		banner;
	menubitmap_s	framel;
	menubitmap_s	framer;

	menutext_s		cinematics;
	menutext_s		demos;
	
	menubitmap_s	back;
} videos_t;

static videos_t s_videos;

/*
=================
UI_VideosMenu_Event
=================
*/
void UI_VideosMenu_Event (void* ptr, int event) {
	if( event != QM_ACTIVATED ) {
		return;
	}

	switch( ((menucommon_s*)ptr)->id ) {
	case ID_CINEMATICS:
		UI_CinematicsMenu();
		break;

	case ID_DEMOS:
		UI_DemosMenu();
		break;

	case ID_BACK:
		UI_PopMenu();
		break;
	}
}

/*
===============
UI_VideosMenu_Init
===============
*/
static void UI_VideosMenu_Init( void ) {
	int		y;
	int		numItems;

	UI_VideosMenu_Cache();

	memset( &s_videos, 0, sizeof(s_videos) );
	s_videos.menu.wrapAround = qtrue;
	s_videos.menu.fullscreen = qtrue;

	numItems = 2;

	y = (SCREEN_HEIGHT - numItems*VIDEOS_MENU_VERTICAL_SPACING) / 2;
	s_videos.banner.generic.type			= MTYPE_BTEXT;
	s_videos.banner.generic.x				= 320;
	s_videos.banner.generic.y				= 16;
	s_videos.banner.string					= "VIDEOS";
	s_videos.banner.color					= text_banner_color;
	s_videos.banner.style					= UI_CENTER;

	s_videos.framel.generic.type			= MTYPE_BITMAP;
	s_videos.framel.generic.name			= ART_FRAMEL;
	s_videos.framel.generic.flags			= QMF_INACTIVE;
	s_videos.framel.generic.x				= 0;  
	s_videos.framel.generic.y				= 78;
	s_videos.framel.width  					= 256;
	s_videos.framel.height  				= 329;

	s_videos.framer.generic.type			= MTYPE_BITMAP;
	s_videos.framer.generic.name			= ART_FRAMER;
	s_videos.framer.generic.flags			= QMF_INACTIVE;
	s_videos.framer.generic.x				= 376;
	s_videos.framer.generic.y				= 76;
	s_videos.framer.width  					= 256;
	s_videos.framer.height  				= 334;

	s_videos.back.generic.type				= MTYPE_BITMAP;
	s_videos.back.generic.name				= ART_BACK0;
	s_videos.back.generic.flags				= QMF_LEFT_JUSTIFY|QMF_PULSEIFFOCUS;
	s_videos.back.generic.id				= ID_BACK;
	s_videos.back.generic.callback			= UI_VideosMenu_Event;
	s_videos.back.generic.x					= 0;
	s_videos.back.generic.y					= 480-64;
	s_videos.back.width						= 128;
	s_videos.back.height					= 64;
	s_videos.back.focuspic					= ART_BACK1;

	y += VIDEOS_MENU_VERTICAL_SPACING;
	s_videos.demos.generic.type				= MTYPE_PTEXT;
	s_videos.demos.generic.flags			= QMF_CENTER_JUSTIFY|QMF_PULSEIFFOCUS;
	s_videos.demos.generic.x				= 320;
	s_videos.demos.generic.y				= y;
	s_videos.demos.generic.id				= ID_DEMOS;
	s_videos.demos.generic.callback			= UI_VideosMenu_Event; 
	s_videos.demos.string					= "REPLAYS";
	s_videos.demos.color					= text_big_color;
	s_videos.demos.style					= UI_CENTER | UI_DROPSHADOW;

	y += VIDEOS_MENU_VERTICAL_SPACING;
	s_videos.cinematics.generic.type		= MTYPE_PTEXT;
	s_videos.cinematics.generic.flags		= QMF_CENTER_JUSTIFY|QMF_PULSEIFFOCUS;
	s_videos.cinematics.generic.x			= 320;
	s_videos.cinematics.generic.y			= y;
	s_videos.cinematics.generic.id			= ID_CINEMATICS;
	s_videos.cinematics.generic.callback	= UI_VideosMenu_Event; 
	s_videos.cinematics.string				= "CINEMATICS";
	s_videos.cinematics.color				= text_big_color;
	s_videos.cinematics.style				= UI_CENTER | UI_DROPSHADOW;

	s_videos.back.generic.type				= MTYPE_BITMAP;
	s_videos.back.generic.name				= ART_BACK0;
	s_videos.back.generic.flags				= QMF_LEFT_JUSTIFY|QMF_PULSEIFFOCUS;
	s_videos.back.generic.id				= ID_BACK;
	s_videos.back.generic.callback			= UI_VideosMenu_Event;
	s_videos.back.generic.x					= 0;
	s_videos.back.generic.y					= 480-64;
	s_videos.back.width						= 128;
	s_videos.back.height					= 64;
	s_videos.back.focuspic					= ART_BACK1;

	Menu_AddItem( &s_videos.menu, &s_videos.banner );
	Menu_AddItem( &s_videos.menu, &s_videos.framel );
	Menu_AddItem( &s_videos.menu, &s_videos.framer );
	Menu_AddItem( &s_videos.menu, &s_videos.demos );
	Menu_AddItem( &s_videos.menu, &s_videos.cinematics );
	Menu_AddItem( &s_videos.menu, &s_videos.back );
}

/*
=================
UI_VideosMenu_Cache
=================
*/
void UI_VideosMenu_Cache( void ) {
	trap_R_RegisterShaderNoMip( ART_BACK0 );
	trap_R_RegisterShaderNoMip( ART_BACK1 );
	trap_R_RegisterShaderNoMip( ART_FRAMEL );
	trap_R_RegisterShaderNoMip( ART_FRAMER );
}

/*
===============
UI_VideosMenu
===============
*/
void UI_VideosMenu( void ) {
	UI_VideosMenu_Init();
	UI_PushMenu( &s_videos.menu );
}
