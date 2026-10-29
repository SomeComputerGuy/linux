// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * KUnit tests for the V4L2 HEVC encode core
 *
 * Runs short sequences of pictures through v4l2_hevc_enc_step() and
 * v4l2_hevc_enc_complete() with a real control handler and a fake CAPTURE
 * buffer, and checks the reference picture set, reference lists, decoded
 * picture buffer, headers and feedback that result.
 */

#include <kunit/test.h>
#include <linux/module.h>
#include <linux/v4l2-controls.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-hevc.h>
#include <media/v4l2-hevc-enc.h>
#include <media/videobuf2-v4l2.h>

#define BUF_SIZE	4096

struct hevc_enc_test {
	struct v4l2_ctrl_handler hdl;
	struct v4l2_hevc_enc enc;
	struct v4l2_pix_format fmt;
	struct v4l2_fract tpf;
	struct vb2_queue queue;
	struct vb2_v4l2_buffer buf;
	u8 *data;
	unsigned int rec_allocated;
};

static void *fake_vaddr(struct vb2_buffer *vb, void *buf_priv)
{
	return buf_priv;
}

static const struct vb2_mem_ops fake_mem_ops = {
	.vaddr = fake_vaddr,
};

static int fake_rec_alloc(struct v4l2_hevc_enc *enc,
			  struct v4l2_hevc_enc_rec_buffer *buffer)
{
	struct hevc_enc_test *t = enc->private_data;

	t->rec_allocated++;
	buffer->private_data = (void *)(uintptr_t)t->rec_allocated;

	return 0;
}

static void fake_rec_free(struct v4l2_hevc_enc *enc,
			  struct v4l2_hevc_enc_rec_buffer *buffer)
{
	struct hevc_enc_test *t = enc->private_data;

	t->rec_allocated--;
}

static const struct v4l2_hevc_enc_ops fake_ops = {
	.rec_buffer_alloc = fake_rec_alloc,
	.rec_buffer_free = fake_rec_free,
};

static const u32 compound_ctrls[] = {
	V4L2_CID_STATELESS_HEVC_SPS,
	V4L2_CID_STATELESS_HEVC_PPS,
	V4L2_CID_STATELESS_HEVC_ENCODE_PARAMS,
	V4L2_CID_STATELESS_HEVC_ENCODE_FEEDBACK,
};

static const struct v4l2_ctrl_hevc_sps test_sps = {
	.pic_width_in_luma_samples = 1280,
	.pic_height_in_luma_samples = 720,
	.log2_max_pic_order_cnt_lsb_minus4 = 4,
	.sps_max_dec_pic_buffering_minus1 = 2,
	.log2_diff_max_min_luma_coding_block_size = 3,
	.log2_diff_max_min_luma_transform_block_size = 3,
	.chroma_format_idc = 1,
	.sps_max_sub_layers_minus1 = 1,
	.flags = V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED |
		 V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT,
};

static struct hevc_enc_test *hevc_enc_test_init(struct kunit *test,
						unsigned int flags)
{
	struct hevc_enc_test *t;
	unsigned int i;
	int ret;

	t = kunit_kzalloc(test, sizeof(*t), GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t);
	t->data = kunit_kzalloc(test, BUF_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, t->data);

	v4l2_ctrl_handler_init(&t->hdl, 8);
	for (i = 0; i < ARRAY_SIZE(compound_ctrls); i++) {
		struct v4l2_ctrl_config cfg = { .id = compound_ctrls[i] };

		v4l2_ctrl_new_custom(&t->hdl, &cfg, NULL);
	}
	v4l2_ctrl_new_std(&t->hdl, NULL, V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
			  0, 51, 1, 30);
	v4l2_ctrl_new_std(&t->hdl, NULL, V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
			  0, 51, 1, 33);
	KUNIT_ASSERT_EQ(test, t->hdl.error, 0);

	ret = v4l2_ctrl_s_ctrl_compound(v4l2_ctrl_find(&t->hdl,
						       V4L2_CID_STATELESS_HEVC_SPS),
					V4L2_CTRL_TYPE_HEVC_SPS, &test_sps);
	KUNIT_ASSERT_EQ(test, ret, 0);

	t->fmt.width = 1280;
	t->fmt.height = 720;
	t->tpf = (struct v4l2_fract){ 1, 30 };

	t->queue.mem_ops = &fake_mem_ops;
	t->buf.vb2_buf.vb2_queue = &t->queue;
	t->buf.vb2_buf.num_planes = 1;
	t->buf.vb2_buf.planes[0].mem_priv = t->data;
	t->buf.vb2_buf.planes[0].length = BUF_SIZE;

	t->enc.ops = &fake_ops;
	t->enc.private_data = t;
	t->enc.format = &t->fmt;
	t->enc.timeperframe = &t->tpf;
	t->enc.ctrl_handler = &t->hdl;
	t->enc.flags = flags;

	ret = v4l2_hevc_enc_init(&t->enc);
	KUNIT_ASSERT_EQ(test, ret, 0);

	return t;
}

static void hevc_enc_test_exit(struct hevc_enc_test *t)
{
	v4l2_hevc_enc_exit(&t->enc);
	v4l2_ctrl_handler_free(&t->hdl);
}

static int hevc_enc_test_set(struct hevc_enc_test *t,
			     const struct v4l2_ctrl_hevc_encode_params *p)
{
	return v4l2_ctrl_s_ctrl_compound(v4l2_ctrl_find(&t->hdl,
							V4L2_CID_STATELESS_HEVC_ENCODE_PARAMS),
					 V4L2_CTRL_TYPE_HEVC_ENCODE_PARAMS, p);
}

/* Encode one picture, with the hardware "writing" @hw_size bytes. */
static int hevc_enc_test_frame(struct kunit *test, struct hevc_enc_test *t,
			       const struct v4l2_ctrl_hevc_encode_params *p,
			       u64 timestamp, unsigned int hw_size)
{
	int ret;

	ret = hevc_enc_test_set(t, p);
	KUNIT_ASSERT_EQ(test, ret, 0);

	memset(t->data, 0, BUF_SIZE);
	t->buf.vb2_buf.timestamp = timestamp;

	ret = v4l2_hevc_enc_step(&t->enc, &t->buf);
	if (ret)
		return ret;

	vb2_set_plane_payload(&t->buf.vb2_buf, 0,
			      t->enc.header_size + hw_size);

	return v4l2_hevc_enc_complete(&t->enc, &t->buf, NULL);
}

static const struct v4l2_ctrl_hevc_encode_feedback *
hevc_enc_test_feedback(struct hevc_enc_test *t)
{
	return v4l2_ctrl_find(&t->hdl, V4L2_CID_STATELESS_HEVC_ENCODE_FEEDBACK)
		->p_cur.p_hevc_encode_feedback;
}

static unsigned int hevc_enc_test_active(struct hevc_enc_test *t)
{
	unsigned int i, count = 0;

	for (i = 0; i < t->enc.ref.slots_count; i++)
		count += t->enc.ref.dpb[i].active;

	return count;
}

static int hevc_enc_test_slot(struct hevc_enc_test *t, u64 timestamp)
{
	unsigned int i;

	for (i = 0; i < t->enc.ref.slots_count; i++)
		if (t->enc.ref.dpb[i].active &&
		    t->enc.ref.dpb[i].timestamp == timestamp)
			return i;

	return -1;
}

/* Like VEPU580 slice header registers, without long-term pictures. */
static const struct v4l2_hevc_enc_rps_limits no_lt_limits = {
	.num_negative_pics = 4,
};

#define REF(ts, f)	{ .timestamp = (ts), .flags = (f) }
#define USED		V4L2_HEVC_ENCODE_REF_FLAG_USED_BY_CURR_PIC
#define LT		V4L2_HEVC_ENCODE_REF_FLAG_LONG_TERM

/* IDR, then P pictures that keep, drop and turn references long-term. */
static void hevc_enc_gop(struct kunit *test)
{
	struct hevc_enc_test *t =
		hevc_enc_test_init(test, V4L2_HEVC_ENC_FLAG_INTER_PRED);
	const struct v4l2_ctrl_hevc_encode_feedback *fb;
	struct v4l2_hevc_slice_header *sh = &t->enc.slice;
	struct v4l2_ctrl_hevc_encode_params p = {
		.nal_unit_type = V4L2_HEVC_NAL_IDR_W_RADL,
		.nuh_temporal_id_plus1 = 1,
		.slice_type = V4L2_HEVC_SLICE_TYPE_I,
		.flags = V4L2_HEVC_ENCODE_FLAG_REFERENCE,
	};
	static const u8 vps_start[] = { 0, 0, 0, 1, 0x40, 0x01 };
	int ret;

	/* POC 0: IDR, with the parameter sets. */
	ret = hevc_enc_test_frame(test, t, &p, 1000, 100);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_GT(test, t->enc.header_size, 40);
	KUNIT_EXPECT_MEMEQ(test, t->data, vps_start, sizeof(vps_start));
	KUNIT_EXPECT_EQ(test, t->enc.qp, 30);
	KUNIT_EXPECT_TRUE(test, t->buf.flags & V4L2_BUF_FLAG_KEYFRAME);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 1);
	fb = hevc_enc_test_feedback(t);
	KUNIT_EXPECT_EQ(test, fb->timestamp, 1000);
	KUNIT_EXPECT_EQ(test, fb->slice_qp, 30);
	KUNIT_EXPECT_EQ(test, fb->stats.header_size, t->enc.header_size);
	KUNIT_EXPECT_TRUE(test,
			  fb->flags & V4L2_HEVC_ENCODE_FEEDBACK_FLAG_REFERENCE);

	/* The current picture and the 3 DPB slots of the SPS. */
	KUNIT_EXPECT_EQ(test, t->rec_allocated, 4);

	/* POC 1: P referencing POC 0. Only a slice header is written. */
	p.nal_unit_type = V4L2_HEVC_NAL_TRAIL_R;
	p.slice_type = V4L2_HEVC_SLICE_TYPE_P;
	p.pic_order_cnt_val = 1;
	p.num_refs = 1;
	p.refs[0] = (struct v4l2_hevc_encode_ref)REF(1000, USED);
	p.flags |= V4L2_HEVC_ENCODE_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
	ret = hevc_enc_test_frame(test, t, &p, 2000, 100);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, t->enc.qp, 33);
	KUNIT_EXPECT_EQ(test, t->data[4] >> 1, V4L2_HEVC_NAL_TRAIL_R);
	KUNIT_EXPECT_LT(test, t->enc.header_size, 16);
	KUNIT_EXPECT_EQ(test, sh->st_rps.num_negative_pics, 1);
	KUNIT_EXPECT_EQ(test, sh->st_rps.delta_poc_s0_minus1[0], 0);
	KUNIT_EXPECT_EQ(test, sh->st_rps.used_by_curr_pic_s0, 0x1);
	KUNIT_EXPECT_EQ(test, sh->slice_pic_order_cnt_lsb, 1);
	KUNIT_EXPECT_EQ(test, sh->slice_qp_delta, 33 - 26);
	KUNIT_EXPECT_EQ(test, t->enc.ref.l0_count, 1);
	KUNIT_EXPECT_EQ(test, t->enc.ref.collocated, t->enc.ref.l0[0]);
	KUNIT_EXPECT_TRUE(test, t->buf.flags & V4L2_BUF_FLAG_PFRAME);
	fb = hevc_enc_test_feedback(t);
	KUNIT_EXPECT_EQ(test, fb->num_ref_idx_l0_active, 1);
	KUNIT_EXPECT_EQ(test, fb->ref_idx_l0[0], 0);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 2);

	/*
	 * POC 2: keep POC 0 without using it, use POC 1. The closest
	 * picture comes first in the RPS whatever the order of refs[].
	 */
	p.pic_order_cnt_val = 2;
	p.num_refs = 2;
	p.refs[0] = (struct v4l2_hevc_encode_ref)REF(1000, 0);
	p.refs[1] = (struct v4l2_hevc_encode_ref)REF(2000, USED);
	ret = hevc_enc_test_frame(test, t, &p, 3000, 100);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, sh->st_rps.num_negative_pics, 2);
	KUNIT_EXPECT_EQ(test, sh->st_rps.delta_poc_s0_minus1[0], 0);
	KUNIT_EXPECT_EQ(test, sh->st_rps.delta_poc_s0_minus1[1], 0);
	KUNIT_EXPECT_EQ(test, sh->st_rps.used_by_curr_pic_s0, 0x1);
	KUNIT_EXPECT_EQ(test, t->enc.ref.l0[0], hevc_enc_test_slot(t, 2000));
	KUNIT_EXPECT_EQ(test, hevc_enc_test_feedback(t)->ref_idx_l0[0], 1);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 3);

	/* POC 3: non-reference, uses POC 2, drops POC 1, POC 0 long-term. */
	p.nal_unit_type = V4L2_HEVC_NAL_TRAIL_N;
	p.pic_order_cnt_val = 3;
	p.refs[0] = (struct v4l2_hevc_encode_ref)REF(3000, USED);
	p.refs[1] = (struct v4l2_hevc_encode_ref)REF(1000, LT);
	p.flags &= ~V4L2_HEVC_ENCODE_FLAG_REFERENCE;
	ret = hevc_enc_test_frame(test, t, &p, 4000, 100);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, sh->st_rps.num_negative_pics, 1);
	KUNIT_EXPECT_EQ(test, sh->num_long_term_pics, 1);
	KUNIT_EXPECT_EQ(test, sh->lt[0].poc_lsb_lt, 0);
	KUNIT_EXPECT_FALSE(test, sh->lt[0].used_by_curr_pic_lt);
	KUNIT_EXPECT_EQ(test, sh->lt[0].delta_poc_msb_cycle_lt, 0);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 2);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_slot(t, 2000), -1);
	KUNIT_EXPECT_TRUE(test,
			  t->enc.ref.dpb[hevc_enc_test_slot(t, 1000)].long_term);
	KUNIT_EXPECT_FALSE(test, hevc_enc_test_feedback(t)->flags &
			   V4L2_HEVC_ENCODE_FEEDBACK_FLAG_REFERENCE);

	/* A long-term picture cannot be used as short-term again. */
	p.pic_order_cnt_val = 4;
	p.num_refs = 2;
	p.refs[0] = (struct v4l2_hevc_encode_ref)REF(3000, USED);
	p.refs[1] = (struct v4l2_hevc_encode_ref)REF(1000, 0);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_frame(test, t, &p, 5000, 100),
			-EINVAL);

	/* A picture that is not in the DPB (dropped, or never encoded). */
	p.refs[1] = (struct v4l2_hevc_encode_ref)REF(2000, 0);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_frame(test, t, &p, 5000, 100),
			-EINVAL);

	/* A POC too far from the previous TemporalId 0 picture. */
	p.num_refs = 1;
	p.pic_order_cnt_val = 2 + 128;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_frame(test, t, &p, 5000, 100),
			-EINVAL);

	/* More references than the hardware can signal. */
	p.pic_order_cnt_val = 4;
	p.num_refs = 2;
	p.refs[1] = (struct v4l2_hevc_encode_ref)REF(1000, LT);
	t->enc.rps_limits = &no_lt_limits;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_frame(test, t, &p, 5000, 100),
			-EINVAL);
	t->enc.rps_limits = NULL;
	p.num_refs = 1;

	/* Failed runs left the DPB alone. */
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 2);

	/* No B slices without INTER_BIPRED. */
	p.pic_order_cnt_val = 4;
	p.slice_type = V4L2_HEVC_SLICE_TYPE_B;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_frame(test, t, &p, 5000, 100),
			-EINVAL);

	/* A CRA keeps references for leading pictures but uses none. */
	p.nal_unit_type = V4L2_HEVC_NAL_CRA_NUT;
	p.slice_type = V4L2_HEVC_SLICE_TYPE_I;
	p.pic_order_cnt_val = 8;
	p.refs[0] = (struct v4l2_hevc_encode_ref)REF(3000, 0);
	p.flags |= V4L2_HEVC_ENCODE_FLAG_REFERENCE;
	ret = hevc_enc_test_frame(test, t, &p, 9000, 100);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, sh->st_rps.num_negative_pics, 1);
	KUNIT_EXPECT_EQ(test, sh->st_rps.delta_poc_s0_minus1[0], 5);
	KUNIT_EXPECT_EQ(test, sh->st_rps.used_by_curr_pic_s0, 0);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 2);

	/* An IDR empties the DPB. */
	p.nal_unit_type = V4L2_HEVC_NAL_IDR_N_LP;
	ret = hevc_enc_test_frame(test, t, &p, 10000, 100);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_active(t), 1);
	KUNIT_EXPECT_GE(test, hevc_enc_test_slot(t, 10000), 0);

	hevc_enc_test_exit(t);
	KUNIT_EXPECT_EQ(test, t->rec_allocated, 0);
}

/*
 * Hardware that writes its own slice headers without TemporalId: the core
 * patches nuh_temporal_id_plus1 and writes nothing for pictures without
 * parameter sets.
 */
static void hevc_enc_hw_slice_header(struct kunit *test)
{
	struct hevc_enc_test *t =
		hevc_enc_test_init(test, V4L2_HEVC_ENC_FLAG_INTER_PRED |
				   V4L2_HEVC_ENC_FLAG_HW_SLICE_HEADER);
	struct v4l2_ctrl_hevc_encode_params p = {
		.nal_unit_type = V4L2_HEVC_NAL_IDR_W_RADL,
		.nuh_temporal_id_plus1 = 1,
		.slice_type = V4L2_HEVC_SLICE_TYPE_I,
		.flags = V4L2_HEVC_ENCODE_FLAG_REFERENCE,
	};
	static const u8 hw_slice[] = { 0, 0, 0, 1, 0x04, 0x01, 0xd0, 0x00,
				       0x00, 0x01, 0x04, 0x01, 0x80 };
	static const u8 patched[] = { 0, 0, 0, 1, 0x04, 0x02, 0xd0, 0x00,
				      0x00, 0x01, 0x04, 0x02, 0x80 };
	unsigned int header_size;
	int ret;

	ret = hevc_enc_test_frame(test, t, &p, 1000, 0);
	KUNIT_ASSERT_EQ(test, ret, 0);
	header_size = t->enc.header_size;
	KUNIT_EXPECT_GT(test, header_size, 0);

	/* Parameter sets only: the buffer ends with the PPS trailing bits. */
	KUNIT_EXPECT_NE(test, t->data[header_size - 1], 0);

	p.nal_unit_type = V4L2_HEVC_NAL_TSA_N;
	p.nuh_temporal_id_plus1 = 2;
	p.slice_type = V4L2_HEVC_SLICE_TYPE_P;
	p.pic_order_cnt_val = 1;
	p.num_refs = 1;
	p.refs[0] = (struct v4l2_hevc_encode_ref)REF(1000, USED);
	p.flags = 0;
	ret = hevc_enc_test_set(t, &p);
	KUNIT_ASSERT_EQ(test, ret, 0);

	t->buf.vb2_buf.timestamp = 2000;
	ret = v4l2_hevc_enc_step(&t->enc, &t->buf);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, t->enc.header_size, 0);

	memcpy(t->data, hw_slice, sizeof(hw_slice));
	vb2_set_plane_payload(&t->buf.vb2_buf, 0, sizeof(hw_slice));
	ret = v4l2_hevc_enc_complete(&t->enc, &t->buf, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_MEMEQ(test, t->data, patched, sizeof(patched));

	hevc_enc_test_exit(t);
}

/* Intra-only hardware turns P pictures into I pictures and keeps no DPB. */
static void hevc_enc_intra_only(struct kunit *test)
{
	struct hevc_enc_test *t = hevc_enc_test_init(test, 0);
	struct v4l2_ctrl_hevc_encode_params p = {
		.nal_unit_type = V4L2_HEVC_NAL_IDR_W_RADL,
		.nuh_temporal_id_plus1 = 1,
		.slice_type = V4L2_HEVC_SLICE_TYPE_I,
		.flags = V4L2_HEVC_ENCODE_FLAG_REFERENCE,
	};
	int ret;

	ret = hevc_enc_test_frame(test, t, &p, 1000, 10);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, t->rec_allocated, 1);
	KUNIT_EXPECT_EQ(test, t->enc.ref.slots_count, 0);

	/* The SPS was accommodated: no DPB, no TMVP. */
	KUNIT_EXPECT_EQ(test,
			v4l2_ctrl_find(&t->hdl, V4L2_CID_STATELESS_HEVC_SPS)
			->p_cur.p_hevc_sps->sps_max_dec_pic_buffering_minus1,
			0);

	p.nal_unit_type = V4L2_HEVC_NAL_TRAIL_R;
	p.slice_type = V4L2_HEVC_SLICE_TYPE_P;
	p.pic_order_cnt_val = 1;
	ret = hevc_enc_test_set(t, &p);
	/* A P picture needs a used reference... */
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);

	p.slice_type = V4L2_HEVC_SLICE_TYPE_I;
	ret = hevc_enc_test_frame(test, t, &p, 2000, 10);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, t->enc.slice.slice_type, V4L2_HEVC_SLICE_TYPE_I);
	KUNIT_EXPECT_EQ(test, t->enc.slice.st_rps.num_negative_pics, 0);

	hevc_enc_test_exit(t);
}

/* Control validation of the encode parameters. */
static void hevc_enc_validate(struct kunit *test)
{
	struct hevc_enc_test *t =
		hevc_enc_test_init(test, V4L2_HEVC_ENC_FLAG_INTER_PRED);
	const struct v4l2_ctrl_hevc_encode_params *cur =
		v4l2_ctrl_find(&t->hdl, V4L2_CID_STATELESS_HEVC_ENCODE_PARAMS)
		->p_cur.p_hevc_encode_params;
	struct v4l2_ctrl_hevc_encode_params p = {
		.nal_unit_type = V4L2_HEVC_NAL_TRAIL_R,
		.nuh_temporal_id_plus1 = 1,
		.slice_type = V4L2_HEVC_SLICE_TYPE_P,
		.num_refs = 1,
		.refs = { REF(1, USED) },
	};
	struct v4l2_ctrl_hevc_encode_params bad;

	/* The default value is a valid IDR picture. */
	KUNIT_EXPECT_EQ(test, cur->nal_unit_type, V4L2_HEVC_NAL_IDR_W_RADL);
	KUNIT_EXPECT_EQ(test, cur->nuh_temporal_id_plus1, 1);

	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &p), 0);
	/* collocated_from_l0_flag is inferred for P slices. */
	KUNIT_EXPECT_TRUE(test, cur->flags &
			  V4L2_HEVC_ENCODE_FLAG_COLLOCATED_FROM_L0);

	bad = p;
	bad.nal_unit_type = V4L2_HEVC_NAL_VPS;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	bad = p;
	bad.nal_unit_type = V4L2_HEVC_NAL_CRA_NUT;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	bad = p;
	bad.nal_unit_type = V4L2_HEVC_NAL_TSA_R;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	bad = p;
	bad.nuh_temporal_id_plus1 = 0;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	bad = p;
	bad.refs[0].flags = 0;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	bad = p;
	bad.num_refs = 2;
	bad.refs[1] = bad.refs[0];
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	bad = p;
	bad.flags = BIT_ULL(40);
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), -EINVAL);

	/* An IDR has no RPS: the references and POC are dropped. */
	bad = p;
	bad.nal_unit_type = V4L2_HEVC_NAL_IDR_N_LP;
	bad.slice_type = V4L2_HEVC_SLICE_TYPE_I;
	bad.refs[0].flags = 0;
	bad.pic_order_cnt_val = 5;
	KUNIT_EXPECT_EQ(test, hevc_enc_test_set(t, &bad), 0);
	KUNIT_EXPECT_EQ(test, cur->num_refs, 0);
	KUNIT_EXPECT_EQ(test, cur->pic_order_cnt_val, 0);
	KUNIT_EXPECT_EQ(test, cur->refs[0].timestamp, 0);

	/* The first picture must be an IRAP picture. */
	KUNIT_EXPECT_EQ(test, hevc_enc_test_frame(test, t, &p, 1, 10),
			-EINVAL);

	hevc_enc_test_exit(t);
}

static struct kunit_case hevc_enc_test_cases[] = {
	KUNIT_CASE(hevc_enc_gop),
	KUNIT_CASE(hevc_enc_hw_slice_header),
	KUNIT_CASE(hevc_enc_intra_only),
	KUNIT_CASE(hevc_enc_validate),
	{}
};

static struct kunit_suite hevc_enc_test_suite = {
	.name = "v4l2-hevc-enc",
	.test_cases = hevc_enc_test_cases,
};

kunit_test_suite(hevc_enc_test_suite);

MODULE_DESCRIPTION("KUnit tests for the V4L2 HEVC encode core");
MODULE_LICENSE("GPL");
