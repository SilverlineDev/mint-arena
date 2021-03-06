/*
===========================================================================
Copyright (C) 1999-2010 id Software LLC, a ZeniMax Media company.

This file is part of Spearmint Source Code.

Spearmint Source Code is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 3 of the License,
or (at your option) any later version.

Spearmint Source Code is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Spearmint Source Code.  If not, see <http://www.gnu.org/licenses/>.

In addition, Spearmint Source Code is also subject to certain additional terms.
You should have received a copy of these additional terms immediately following
the terms and conditions of the GNU General Public License.  If not, please
request a copy in writing from id Software at the address below.

If you have questions concerning this license or the applicable additional
terms, you may contact in writing id Software LLC, c/o ZeniMax Media Inc.,
Suite 120, Rockville, Maryland 20850 USA.
===========================================================================
*/
//
// cg_drawtools.c -- helper functions called by cg_draw, cg_scoreboard, cg_info, etc
#include "cg_local.h"

static screenPlacement_e cg_horizontalPlacement = PLACE_CENTER;
static screenPlacement_e cg_verticalPlacement = PLACE_CENTER;
static screenPlacement_e cg_lastHorizontalPlacement = PLACE_CENTER;
static screenPlacement_e cg_lastVerticalPlacement = PLACE_CENTER;

/*
================
CG_SetScreenPlacement
================
*/
void CG_SetScreenPlacement(screenPlacement_e hpos, screenPlacement_e vpos)
{
	cg_lastHorizontalPlacement = cg_horizontalPlacement;
	cg_lastVerticalPlacement = cg_verticalPlacement;

	cg_horizontalPlacement = hpos;
	cg_verticalPlacement = vpos;
}

/*
================
CG_PopScreenPlacement
================
*/
void CG_PopScreenPlacement(void)
{
	cg_horizontalPlacement = cg_lastHorizontalPlacement;
	cg_verticalPlacement = cg_lastVerticalPlacement;
}

/*
================
CG_GetScreenHorizontalPlacement
================
*/
screenPlacement_e CG_GetScreenHorizontalPlacement(void)
{
	return cg_horizontalPlacement;
}

/*
================
CG_GetScreenVerticalPlacement
================
*/
screenPlacement_e CG_GetScreenVerticalPlacement(void)
{
	return cg_verticalPlacement;
}

/*
================
CG_AdjustFrom640

Adjusted for resolution and screen aspect ratio
================
*/
void CG_AdjustFrom640( float *x, float *y, float *w, float *h ) {
	int viewXBias = 0;

	if (cg.numViewports != 1 && cg.snap && ( x != NULL || y != NULL ) ) {
		qboolean right = qfalse;
		qboolean down = qfalse;

		if (cg.numViewports == 2) {
			if (cg.viewport == 1) {
				down = qtrue;
			}
		}
		else if (cg.numViewports == 3 && cg.viewport == 2) {
			down = qtrue;
		}
		else if (cg.numViewports <= 4) {
			if (cg.viewport == 1 || cg.viewport == 3) {
				right = qtrue;
			}
			if (cg.viewport == 2 || cg.viewport == 3) {
				down = qtrue;
			}
		}

		if (cg.viewport != 0 && (cg.numViewports == 2 || cg.numViewports == 3) && cg_splitviewVertical.integer) {
			right = !right;
			down = !down;
		}

		if (right) {
			viewXBias = 2;
			if ( x != NULL ) {
				*x += SCREEN_WIDTH;
			}
		}
		if (down) {
			if ( y != NULL ) {
				*y += SCREEN_HEIGHT;
			}
		}
	}

	if (cg_horizontalPlacement == PLACE_STRETCH) {
		// scale for screen sizes (not aspect correct in wide screen)
		if ( w != NULL ) {
			*w *= cgs.screenXScaleStretch;
		}
		if ( x != NULL ) {
			*x *= cgs.screenXScaleStretch;
		}
	} else {
		// scale for screen sizes
		if ( w != NULL ) {
			*w *= cgs.screenXScale;
		}

		if ( x != NULL ) {
			*x *= cgs.screenXScale;

			// Screen Placement
			if (cg_horizontalPlacement == PLACE_CENTER) {
				*x += cgs.screenXBias;
			} else if (cg_horizontalPlacement == PLACE_RIGHT) {
				*x += cgs.screenXBias*2;
			}

			// Offset for widescreen
			*x += cgs.screenXBias*(viewXBias);
		}
	}

	if (cg_verticalPlacement == PLACE_STRETCH) {
		if ( h != NULL ) {
			*h *= cgs.screenYScaleStretch;
		}
		if ( y != NULL ) {
			*y *= cgs.screenYScaleStretch;
		}
	} else {
		if ( h != NULL ) {
			*h *= cgs.screenYScale;
		}

		if ( y != NULL ) {
			*y *= cgs.screenYScale;

			if (cg_verticalPlacement == PLACE_CENTER) {
				*y += cgs.screenYBias;
			} else if (cg_verticalPlacement == PLACE_BOTTOM) {
				*y += cgs.screenYBias*2;
			}
		}
	}
}

/*
================
CG_FillRect

Coordinates are 640*480 virtual values
=================
*/
void CG_FillRect( float x, float y, float width, float height, const float *color ) {
	trap_R_SetColor( color );

	CG_AdjustFrom640( &x, &y, &width, &height );
	trap_R_DrawStretchPic( x, y, width, height, 0, 0, 0, 0, cgs.media.whiteShader );

	trap_R_SetColor( NULL );
}

/*
================
CG_DrawSides

Coords are virtual 640x480
================
*/
void CG_DrawSides(float x, float y, float w, float h, float size) {
	CG_AdjustFrom640( &x, &y, &w, &h );
	if (cg_horizontalPlacement == PLACE_STRETCH) {
		size *= cgs.screenXScaleStretch;
	} else {
		size *= cgs.screenXScale;
	}
	trap_R_DrawStretchPic( x, y, size, h, 0, 0, 0, 0, cgs.media.whiteShader );
	trap_R_DrawStretchPic( x + w - size, y, size, h, 0, 0, 0, 0, cgs.media.whiteShader );
}

void CG_DrawTopBottom(float x, float y, float w, float h, float size) {
	CG_AdjustFrom640( &x, &y, &w, &h );
	if (cg_verticalPlacement == PLACE_STRETCH) {
		size *= cgs.screenYScaleStretch;
	} else {
		size *= cgs.screenYScale;
	}
	trap_R_DrawStretchPic( x, y, w, size, 0, 0, 0, 0, cgs.media.whiteShader );
	trap_R_DrawStretchPic( x, y + h - size, w, size, 0, 0, 0, 0, cgs.media.whiteShader );
}
/*
================
CG_DrawRect

Coordinates are 640*480 virtual values
=================
*/
void CG_DrawRect( float x, float y, float width, float height, float size, const float *color ) {
	trap_R_SetColor( color );

  CG_DrawTopBottom(x, y, width, height, size);
  CG_DrawSides(x, y+size, width, height-size*2, size);

	trap_R_SetColor( NULL );
}

/*
================
CG_ClearViewport

Wide and narrow aspect ratios screens need to have the sides cleared.
Used when drawing fullscreen 4:3 UI.
=================
*/
void CG_ClearViewport( void ) {
	trap_R_SetColor( g_color_table[0] );
	trap_R_DrawStretchPic( cg.viewportX, cg.viewportY, cg.viewportWidth, cg.viewportHeight, 0, 0, 0, 0, cgs.media.whiteShader );
	trap_R_SetColor( NULL );
}



/*
================
CG_DrawPic

Coordinates are 640*480 virtual values
=================
*/
void CG_DrawPic( float x, float y, float width, float height, qhandle_t hShader ) {
	float	s0;
	float	s1;
	float	t0;
	float	t1;

	if( width < 0 ) {	// flip about vertical
		width  = -width;
		s0 = 1;
		s1 = 0;
	}
	else {
		s0 = 0;
		s1 = 1;
	}

	if( height < 0 ) {	// flip about horizontal
		height  = -height;
		t0 = 1;
		t1 = 0;
	}
	else {
		t0 = 0;
		t1 = 1;
	}

	CG_AdjustFrom640( &x, &y, &width, &height );
	trap_R_DrawStretchPic( x, y, width, height, s0, t0, s1, t1, hShader );
}

/*
================
CG_DrawNamedPic

Coordinates are 640*480 virtual values
=================
*/
void CG_DrawNamedPic( float x, float y, float width, float height, const char *picname ) {
	CG_DrawPic( x, y, width, height, trap_R_RegisterShaderNoMip( picname ) );
}

/*
================
CG_SetClipRegion
=================
*/
void CG_SetClipRegion( float x, float y, float w, float h ) {
	vec4_t clip;

	CG_AdjustFrom640( &x, &y, &w, &h );

	clip[ 0 ] = x;
	clip[ 1 ] = y;
	clip[ 2 ] = x + w;
	clip[ 3 ] = y + h;

	trap_R_SetClipRegion( clip );
}

/*
================
CG_ClearClipRegion
=================
*/
void CG_ClearClipRegion( void ) {
	trap_R_SetClipRegion( NULL );
}

/*
=================
CG_LerpColor
=================
*/
void CG_LerpColor( const vec4_t a, const vec4_t b, vec4_t c, float t )
{
	int i;

	// lerp and clamp each component
	for (i=0; i<4; i++)
	{
		c[i] = a[i] + t*(b[i]-a[i]);
		if (c[i] < 0)
			c[i] = 0;
		else if (c[i] > 1.0)
			c[i] = 1.0;
	}
}


/*
==================
CG_DrawString

Draws a multi-colored string with a drop shadow, optionally forcing
to a fixed color.

Coordinates are at 640 by 480 virtual resolution
==================
*/
void CG_DrawString( int x, int y, const char* str, int style, const vec4_t color ) {
	CG_DrawStringExtWithCursor( x, y, str, style, color, 0, 0, 0, -1, -1 );
}
void CG_DrawStringWithCursor( int x, int y, const char* str, int style, const vec4_t color, int cursorPos, int cursorChar ) {
	CG_DrawStringExtWithCursor( x, y, str, style, color, 0, 0, 0, cursorPos, cursorChar );
}
void CG_DrawStringExt( int x, int y, const char* str, int style, const vec4_t color, float scale, int maxChars, float shadowOffset ) {
	CG_DrawStringExtWithCursor( x, y, str, style, color, scale, maxChars, shadowOffset, -1, -1 );
}
void CG_DrawStringExtWithCursor( int x, int y, const char* str, int style, const vec4_t color, float scale, int maxChars, float shadowOffset, int cursorPos, int cursorChar ) {
	int		charh;
	vec4_t	newcolor;
	vec4_t	lowlight;
	const float	*drawcolor;
	const fontInfo_t *font;
	int			decent;

	if( !str ) {
		return;
	}

	if ( !color ) {
		color = colorWhite;
	}

	if ((style & UI_BLINK) && ((cg.realTime/BLINK_DIVISOR) & 1))
		return;

	if (style & UI_TINYFONT)
	{
		font = &cgs.media.tinyFont;
		charh =	TINYCHAR_HEIGHT;
	}
	else if (style & UI_SMALLFONT)
	{
		font = &cgs.media.smallFont;
		charh =	SMALLCHAR_HEIGHT;
	}
	else if (style & UI_GIANTFONT)
	{
		font = &cgs.media.bigFont;
		charh =	GIANTCHAR_HEIGHT;
	}
	else
	{
		font = &cgs.media.textFont;
		charh =	BIGCHAR_HEIGHT;
	}

	if ( scale <= 0 ) {
		scale = charh / 48.0f;
	} else {
		charh = 48 * scale;
	}

	if (style & UI_PULSE)
	{
		lowlight[0] = 0.8*color[0];
		lowlight[1] = 0.8*color[1];
		lowlight[2] = 0.8*color[2];
		lowlight[3] = 0.8*color[3];
		CG_LerpColor(color,lowlight,newcolor,0.5+0.5*sin(cg.realTime/PULSE_DIVISOR));
		drawcolor = newcolor;
	}
	else
		drawcolor = color;

	switch (style & UI_FORMATMASK)
	{
		case UI_CENTER:
			// center justify at x
			x = x - Text_Width( str, font, scale, 0 ) / 2;
			break;

		case UI_RIGHT:
			// right justify at x
			x = x - Text_Width( str, font, scale, 0 );
			break;

		default:
			// left justify at x
			break;
	}

	if ( shadowOffset == 0 && ( style & UI_DROPSHADOW ) ) {
		shadowOffset = 2;
	}

	// This function expects that y is top of line, text_paint expects at baseline
	decent = -font->glyphs[(int)'g'].top + font->glyphs[(int)'g'].height;
	y = y + charh - decent * scale * font->glyphScale;
	if ( cursorChar >= 0 ) {
		Text_PaintWithCursor( x, y, font, scale, drawcolor, str, cursorPos, cursorChar, 0, maxChars, shadowOffset, ( style & UI_FORCECOLOR ) );
	} else {
		Text_Paint( x, y, font, scale, drawcolor, str, 0, maxChars, shadowOffset, ( style & UI_FORCECOLOR ) );
	}
}

void CG_DrawBigString( int x, int y, const char *s, float alpha ) {
	float	color[4];

	color[0] = color[1] = color[2] = 1.0;
	color[3] = alpha;
	CG_DrawString( x, y, s, UI_DROPSHADOW|UI_BIGFONT, color );
}

void CG_DrawBigStringColor( int x, int y, const char *s, vec4_t color ) {
	CG_DrawString( x, y, s, UI_FORCECOLOR|UI_DROPSHADOW|UI_BIGFONT, color );
}

void CG_DrawSmallString( int x, int y, const char *s, float alpha ) {
	float	color[4];

	color[0] = color[1] = color[2] = 1.0;
	color[3] = alpha;
	CG_DrawString( x, y, s, UI_SMALLFONT, color );
}

void CG_DrawSmallStringColor( int x, int y, const char *s, vec4_t color ) {
	CG_DrawString( x, y, s, UI_FORCECOLOR|UI_SMALLFONT, color );
}

/*
=================
CG_DrawStrlenEx

Returns draw width, skiping color escape codes
=================
*/
int CG_DrawStrlenEx( const char *str, int style, int maxchars ) {
	const fontInfo_t *font;
	int charh;

	if (style & UI_TINYFONT)
	{
		font = &cgs.media.tinyFont;
		charh =	TINYCHAR_HEIGHT;
	}
	else if (style & UI_SMALLFONT)
	{
		font = &cgs.media.smallFont;
		charh =	SMALLCHAR_HEIGHT;
	}
	else if (style & UI_GIANTFONT)
	{
		font = &cgs.media.bigFont;
		charh =	GIANTCHAR_HEIGHT;
	}
	else
	{
		font = &cgs.media.textFont;
		charh =	BIGCHAR_HEIGHT;
	}

	return Text_Width( str, font, charh / 48.0f, maxchars );
}

/*
=================
CG_DrawStrlen

Returns draw width, skiping color escape codes
=================
*/
int CG_DrawStrlen( const char *str, int style ) {
	const fontInfo_t *font;
	int charh;

	if (style & UI_TINYFONT)
	{
		font = &cgs.media.tinyFont;
		charh =	TINYCHAR_HEIGHT;
	}
	else if (style & UI_SMALLFONT)
	{
		font = &cgs.media.smallFont;
		charh =	SMALLCHAR_HEIGHT;
	}
	else if (style & UI_GIANTFONT)
	{
		font = &cgs.media.bigFont;
		charh =	GIANTCHAR_HEIGHT;
	}
	else
	{
		font = &cgs.media.textFont;
		charh =	BIGCHAR_HEIGHT;
	}

	return Text_Width( str, font, charh / 48.0f, 0 );
}

/*
=============
CG_TileClearBox

This repeats a 64*64 tile graphic to fill the screen around a sized down
refresh window.
=============
*/
static void CG_TileClearBox( int x, int y, int w, int h, qhandle_t hShader ) {
	float	s1, t1, s2, t2;

	s1 = x/64.0;
	t1 = y/64.0;
	s2 = (x+w)/64.0;
	t2 = (y+h)/64.0;
	trap_R_DrawStretchPic( x, y, w, h, s1, t1, s2, t2, hShader );
}



/*
==============
CG_TileClear

Clear around a sized down screen
==============
*/
void CG_TileClear( void ) {
	int		top, bottom, left, right;
	float		x, y, w, h;

	if (cg.cur_ps->pm_type == PM_INTERMISSION || cg_viewsize.integer >= 100) {
		return;		// full screen rendering
	}

	// viewport coords
	x = cg.viewportX;
	y = cg.viewportY;
	w = cg.viewportWidth;
	h = cg.viewportHeight;

	// view screen coords
	top = cg.refdef.y;
	bottom = top + cg.refdef.height-1;
	left = cg.refdef.x;
	right = left + cg.refdef.width-1;

	// clear above view screen
	CG_TileClearBox( x, y, w, top, cgs.media.backTileShader );

	// clear below view screen
	CG_TileClearBox( x, bottom, w, h - bottom, cgs.media.backTileShader );

	// clear left of view screen
	CG_TileClearBox( x, top, left, bottom - top + 1, cgs.media.backTileShader );

	// clear right of view screen
	CG_TileClearBox( right, top, w - right, bottom - top + 1, cgs.media.backTileShader );
}



/*
================
CG_FadeColor
================
*/
float *CG_FadeColor( int startMsec, int totalMsec ) {
	static vec4_t		color;
	int			t;

	if ( startMsec == 0 ) {
		return NULL;
	}

	t = cg.time - startMsec;

	if ( t >= totalMsec ) {
		return NULL;
	}

	// fade out
	if ( totalMsec - t < FADE_TIME ) {
		color[3] = ( totalMsec - t ) * 1.0/FADE_TIME;
	} else {
		color[3] = 1.0;
	}
	color[0] = color[1] = color[2] = 1;

	return color;
}


/*
================
CG_TeamColor
================
*/
float *CG_TeamColor( int team ) {
	static vec4_t	red = {1, 0.2f, 0.2f, 1};
	static vec4_t	blue = {0.2f, 0.2f, 1, 1};
	static vec4_t	other = {1, 1, 1, 1};
	static vec4_t	spectator = {0.7f, 0.7f, 0.7f, 1};

	switch ( team ) {
	case TEAM_RED:
		return red;
	case TEAM_BLUE:
		return blue;
	case TEAM_SPECTATOR:
		return spectator;
	default:
		return other;
	}
}



/*
=================
CG_GetColorForHealth
=================
*/
void CG_GetColorForHealth( int health, int armor, vec4_t hcolor ) {
	int		count;
	int		max;

	// calculate the total points of damage that can
	// be sustained at the current health / armor level
	if ( health <= 0 ) {
		VectorClear( hcolor );	// black
		hcolor[3] = 1;
		return;
	}
	count = armor;
	max = health * ARMOR_PROTECTION / ( 1.0 - ARMOR_PROTECTION );
	if ( max < count ) {
		count = max;
	}
	health += count;

	// set the color based on health
	hcolor[0] = 1.0;
	hcolor[3] = 1.0;
	if ( health >= 100 ) {
		hcolor[2] = 1.0;
	} else if ( health < 66 ) {
		hcolor[2] = 0;
	} else {
		hcolor[2] = ( health - 66 ) / 33.0;
	}

	if ( health > 60 ) {
		hcolor[1] = 1.0;
	} else if ( health < 30 ) {
		hcolor[1] = 0;
	} else {
		hcolor[1] = ( health - 30 ) / 30.0;
	}
}

/*
=================
CG_ColorForHealth
=================
*/
void CG_ColorForHealth( vec4_t hcolor ) {

	CG_GetColorForHealth( cg.cur_ps->stats[STAT_HEALTH], 
		cg.cur_ps->stats[STAT_ARMOR], hcolor );
}

/*
=================
CG_KeysStringForBinding
=================
*/
void CG_KeysStringForBinding(const char *binding, char *string, int stringSize ) {
	char name2[32];
	int keys[2];
	int i, key;

	for ( i = 0, key = 0; i < 2; i++ )
	{
		key = trap_Key_GetKey( binding, key );
		keys[i] = key;
		key++;
	}

	if (keys[0] == -1) {
		Q_strncpyz( string, "???", stringSize );
		return;
	}

	trap_Key_KeynumToStringBuf( keys[0], string, MIN( 32, stringSize ) );
	Q_strupr(string);

	if (keys[1] != -1)
	{
		trap_Key_KeynumToStringBuf( keys[1], name2, 32 );
		Q_strupr(name2);

		Q_strcat( string, stringSize, " or " );
		Q_strcat( string, stringSize, name2 );
	}
}

