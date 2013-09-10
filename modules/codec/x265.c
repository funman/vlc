/*****************************************************************************
 * x265.c: hevc video encoder
 *****************************************************************************
 * Copyright (C) 2004-2013 the VideoLAN team
 * $Id$
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Ilkka Ollakka <ileoo (at)videolan org>
 *          Rafaël Carré <funman@videolanorg>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************/

/*****************************************************************************
 * Preamble
 *****************************************************************************/
#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <vlc_common.h>
#include <vlc_plugin.h>
#include <vlc_sout.h>
#include <vlc_codec.h>
#include <vlc_charset.h>
#include <vlc_cpu.h>
#include <math.h>

#include <x265.h>

#include <assert.h>
/*****************************************************************************
 * Module descriptor
 *****************************************************************************/
static int  Open ( vlc_object_t * );
static void Close( vlc_object_t * );

vlc_module_begin ()
    set_description( N_("H.265/HEVC encoder (x265)"))
    set_capability( "encoder", 200 )
    set_callbacks( Open, Close )
    set_category( CAT_INPUT )
    set_subcategory( SUBCAT_INPUT_VCODEC )
vlc_module_end ()

/*****************************************************************************
 * Local prototypes
 *****************************************************************************/

static block_t *Encode( encoder_t *, picture_t * );

struct encoder_sys_t
{
    x265_t          *h;

    x265_param_t    param;

    mtime_t         i_initial_delay;

    int             i_sei_size;
    uint8_t         *p_sei;

    mtime_t         dts;
    mtime_t         initial_date;
};

/*****************************************************************************
 * Open: probe the encoder
 *****************************************************************************/
static int  Open ( vlc_object_t *p_this )
{
    encoder_t     *p_enc = (encoder_t *)p_this;
    encoder_sys_t *p_sys;
    //int i_val;
    //char *psz_val;
    //int i_qmin = 0, i_qmax = 0;
    x265_nal_t    *nal;
    int i, i_nal;
    //bool fullrange = false;

    if( p_enc->fmt_out.i_codec != VLC_CODEC_HEVC && !p_enc->b_force )
        return VLC_EGENERIC;

    //config_ChainParse( p_enc, SOUT_CFG_PREFIX, ppsz_sout_options, p_enc->p_cfg );

    p_enc->fmt_out.i_cat = VIDEO_ES;
    p_enc->fmt_out.i_codec = VLC_CODEC_HEVC;
    p_enc->p_sys = p_sys = malloc( sizeof( encoder_sys_t ) );
    if( !p_sys )
        return VLC_ENOMEM;

    p_enc->fmt_in.i_codec = VLC_CODEC_I420;
    //p_sys->i_colorspace = X265_CSP_I420;

    p_enc->pf_encode_video = Encode;
    p_enc->pf_encode_audio = NULL;
    p_sys->i_initial_delay = 0;
    p_sys->i_sei_size = 0;
    p_sys->p_sei = NULL;

    x265_param_default(&p_sys->param);

    p_sys->param.frameNumThreads = 8;
    p_sys->param.poolNumThreads = 8;

    p_sys->param.frameRate = p_enc->fmt_in.video.i_frame_rate / 
            p_enc->fmt_in.video.i_frame_rate_base;
    p_sys->param.sourceWidth = p_enc->fmt_in.video.i_visible_width;
    p_sys->param.sourceHeight = p_enc->fmt_in.video.i_visible_height;


    /* Open the encoder */
    p_sys->h = x265_encoder_open( &p_sys->param );

    if( p_sys->h == NULL )
    {
        msg_Err( p_enc, "cannot open x265 encoder" );
        Close( VLC_OBJECT(p_enc) );
        return VLC_EGENERIC;
    }

    /* get the globals headers */
    if (x265_encoder_headers( p_sys->h, &nal, &i_nal )) {
        Close(VLC_OBJECT(p_enc));
        return VLC_EGENERIC;
    }
    size_t i_extra = 0;
    for( i = 0; i < i_nal; i++ )
        i_extra += nal[i].i_payload;

    uint8_t *p_extra = p_enc->fmt_out.p_extra = malloc( i_extra );
    if( !p_extra )
    {
        Close( VLC_OBJECT(p_enc) );
        return VLC_ENOMEM;
    }

    for( i = 0; i < i_nal; i++ )
    {
        memcpy( p_extra, nal[i].p_payload, nal[i].i_payload );
        p_extra += nal[i].i_payload;
    }

    p_enc->fmt_out.i_extra = i_extra;

    p_sys->i_sei_size = i_extra;
    p_sys->p_sei = malloc(i_extra);
    memcpy(p_sys->p_sei, p_enc->fmt_out.p_extra, i_extra);

    p_sys->dts = 0;
    p_sys->initial_date = 0;

    return VLC_SUCCESS;
}

/****************************************************************************
 * Encode:
 ****************************************************************************/
static block_t *Encode( encoder_t *p_enc, picture_t *p_pict )
{
    msg_Dbg(p_enc, "Encoding");
    encoder_sys_t *p_sys = p_enc->p_sys;
    x265_picture_t pic;
    x265_nal_t *nal;
    block_t *p_block;
    int i_nal=0, i_out=0, i=0;

    if( likely(p_pict) ) {
       if (unlikely(p_sys->initial_date == 0))
            p_sys->initial_date = p_pict->date;
       for( i = 0; i < p_pict->i_planes; i++ )
       {
           pic.planes[i] = p_pict->p[i].p_pixels;
           pic.stride[i] = p_pict->p[i].i_pitch;
       }

       x265_encoder_encode( p_sys->h, &nal, &i_nal, &pic, &pic );
    } else 
        x265_encoder_encode( p_sys->h, &nal, &i_nal, NULL, &pic );

    if( !i_nal ) return NULL;


    /* Get size of block we need */
    for( i = 0; i < i_nal; i++ )
        i_out += nal[i].i_payload;

    p_block = block_Alloc( i_out + p_sys->i_sei_size );
    if( !p_block ) return NULL;

    unsigned int i_offset = 0;
    if( unlikely( p_sys->i_sei_size ) )
    {
       /* insert x265 headers SEI nal into the first picture block at the start */
       memcpy( p_block->p_buffer, p_sys->p_sei, p_sys->i_sei_size );
       i_offset = p_sys->i_sei_size;
       p_sys->i_sei_size = 0;
       free( p_sys->p_sei );
       p_sys->p_sei = NULL;
    }
    /* copy encoded data directly to block */
    memcpy( p_block->p_buffer + i_offset, nal[0].p_payload, i_out );

#if 0 // coming soon in x265
    if( pic.b_keyframe )
        p_block->i_flags |= BLOCK_FLAG_TYPE_I;
    else if( pic.i_type == X265_TYPE_P || pic.i_type == X265_TYPE_I )
        p_block->i_flags |= BLOCK_FLAG_TYPE_P;
    else if( IS_X265_TYPE_B( pic.i_type ) )
        p_block->i_flags |= BLOCK_FLAG_TYPE_B;
    else
        p_block->i_flags |= BLOCK_FLAG_TYPE_PB;
#endif

    /* This isn't really valid for streams with B-frames */
    p_block->i_length = CLOCK_FREQ *
        p_enc->fmt_in.video.i_frame_rate_base /
            p_enc->fmt_in.video.i_frame_rate;

    p_block->i_pts = p_sys->initial_date + pic.poc * p_block->i_length;
    p_block->i_dts = p_sys->initial_date + p_sys->dts++ * p_block->i_length;
    msg_Dbg(p_enc, "%zu bytes", p_block->i_buffer);
    msg_Dbg(p_enc, "PTS %lld DTS %lld", p_block->i_pts, p_block->i_dts);

    return p_block;
}

/*****************************************************************************
 * CloseEncoder: x265 encoder destruction
 *****************************************************************************/
static void Close( vlc_object_t *p_this )
{
    encoder_t     *p_enc = (encoder_t *)p_this;
    encoder_sys_t *p_sys = p_enc->p_sys;

    free( p_sys->p_sei );

    if( p_sys->h )
        x265_encoder_close( p_sys->h, NULL );

    free( p_sys );
}
