#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "libavcodec/avcodec.h"
#include "decoder.h"

#if RECOMPRESS
#include <sys/stat.h>
#endif

#define READ_SIZE 4096
#define BUFFER_CAPACITY 4096*64

extern AVCodec ff_h264_decoder;
extern AVCodecParser ff_h264_parser;

static void yuv_save(unsigned char *buf[], int wrap[], int xsize,int ysize, FILE *f)
{
	int i;	
	for (i = 0; i < ysize; i++) {
		fwrite(buf[0] + i * wrap[0], 1, xsize, f);
	}
	for (i = 0; i < ysize / 2; i++) {
		fwrite(buf[1] + i * wrap[1], 1, xsize/2, f);
	}
	for (i = 0; i < ysize / 2; i++) {
		fwrite(buf[2] + i * wrap[2], 1, xsize/2, f);
	}
}

#if RECOMPRESS
static int decode_write_frame(AVCodecContext *avctx,
							  AVFrame *frame, int *frame_index, AVPacket *pkt, int flush)//去掉保存yuv的file
#else
static int decode_write_frame(FILE *file, AVCodecContext *avctx,
							  AVFrame *frame, int *frame_index, AVPacket *pkt, int flush)
#endif
{
	int got_frame = 0;
	do {
		int len = avcodec_decode_video2(avctx, frame, &got_frame, pkt);
		if (len < 0) {
			fprintf(stderr, "Error while decoding frame %d\n", *frame_index);
			return len;
		}
		if (got_frame) {
			printf("Got frame %d\n", *frame_index);
#if RECOMPRESS == 0
			//反向，不开重压缩时保存yuv
			if (file) {
				yuv_save(frame->data, frame->linesize, frame->width, frame->height, file);
			}
#endif
			(*frame_index)++;
		}
	} while (flush && got_frame);
	return 0;
}

#if RECOMPRESS
static void h264_video_decode(const char *filename, const char *reCompressStreamName, const char *deCompressStreamName)//加入重压缩流和解码流
#else
static void h264_video_decode(const char *filename, const char *outfilename)
#endif
{
#if RECOMPRESS
	if(reCompressStreamName != NULL){
		printf("recompress file '%s' to '%s' \n", filename, reCompressStreamName);
	}else if(deCompressStreamName != NULL){
		printf("decompress file '%s' to '%s' \n", filename, deCompressStreamName);
	}else{
		fprintf(stderr, "file errors \n");
		exit(1);
	}
#else
	printf("Decode file '%s' to '%s'\n", filename, outfilename);
#endif
	FILE *file = fopen(filename, "rb");
	if (!file) {
		fprintf(stderr, "Could not open '%s'\n", filename);
		exit(1);
	}

#if RECOMPRESS == 0
	//关掉yuv文件
	FILE *outfile = fopen(outfilename, "wb");
	if (!outfile) {
		fprintf(stderr, "Could not open '%s'\n", outfilename);
		exit(1);
	}
#endif
	avcodec_register(&ff_h264_decoder);
	av_register_codec_parser(&ff_h264_parser);
	
	AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (!codec) {
		fprintf(stderr, "Codec not found\n");
		exit(1);
	}

	AVCodecContext *codec_ctx = avcodec_alloc_context3(codec);
	if (!codec_ctx) {
		fprintf(stderr, "Could not allocate video codec context\n");
		exit(1);
	}
	
	if (avcodec_open2(codec_ctx, codec, NULL) < 0) {
		fprintf(stderr, "Could not open codec\n");
		exit(1);
	}
	
	AVCodecParserContext* parser = av_parser_init(AV_CODEC_ID_H264);
	if(!parser) {
		fprintf(stderr, "Could not create H264 parser\n");
		exit(1);
	}

	AVFrame *frame = av_frame_alloc();
	if (!frame) {
		fprintf(stderr, "Could not allocate video frame\n");
		exit(1);
	}
#if RECOMPRESS 
	codec_ctx->speedup = 1;//跳过不必要的解码
	codec_ctx->reCompressStreamFile = NULL;
	codec_ctx->deCompressStreamFile = NULL;
	if(reCompressStreamName != NULL){
		codec_ctx->compressMode = 1;
		codec_ctx->reCompressStreamFile = fopen(reCompressStreamName, "wb");
		if(!codec_ctx->reCompressStreamFile){//检测打开报错
			fprintf(stderr, "Could not open '%s' \n", reCompressStreamName);
			exit(1);
		}
	}
	else if(deCompressStreamName != NULL){//小问题，else if逻辑
		codec_ctx->compressMode = 2;
		codec_ctx->reCompressStreamFile = fopen(deCompressStreamName, "wb");
		if(!codec_ctx->deCompressStreamFile){//检测打开报错
			fprintf(stderr, "Could not open '%s' \n", deCompressStreamName);
			exit(1);
		}
	}
	else{
		codec_ctx->compressMode = 0;
	}
#endif
	int ending = 0;
	int need_more = 1;
	int frame_index = 0;
	uint8_t buffer[BUFFER_CAPACITY];
	uint8_t* buf = buffer;
	int buf_size = 0;
	AVPacket packet;
	
	struct timeval tv_start, tv_end;
	gettimeofday(&tv_start, NULL);
	while (!ending) {
		if (need_more == 1 && buf_size + READ_SIZE <= BUFFER_CAPACITY) {
			// Move unused data in buffer to front, if any
			if (buf_size > 0) {
				memcpy(buffer, buf, buf_size);
				buf = buffer;
			}
			int bytes_read = fread(buffer + buf_size, 1, READ_SIZE, file);
			if (bytes_read == 0) {
				// EOF or error
				ending = 1;
			} else {
				buf_size += bytes_read;
				need_more = 0;
			}
		}
		
		uint8_t* data = NULL;
  		int size = 0;
		int bytes_used = av_parser_parse2(parser, codec_ctx, &data, &size, buf, buf_size, 0, 0, AV_NOPTS_VALUE);
		if (size == 0) {
			need_more = 1;
			continue;
		}
		if (bytes_used > 0 || ending == 1) {
			// We have data of one packet, decode it; or decode whatever when ending
			av_init_packet(&packet);
			packet.data = data;
			packet.size = size;

#if RECOMPRESS
			int ret = decode_write_frame(codec_ctx, frame, &frame_index, &packet, 0);
#else
			int ret = decode_write_frame(outfile, codec_ctx, frame, &frame_index, &packet, 0);
#endif
			if (ret < 0) {
				fprintf(stderr, "Decode or write frame error\n");
				exit(1);
			}
			
			buf_size -= bytes_used;
			buf += bytes_used;
		}
	}

	// Flush the decoder
	packet.data = NULL;
	packet.size = 0;
#if RECOMPRESS
	decode_write_frame(codec_ctx, frame, &frame_index, &packet, 1);
#else
	decode_write_frame(outfile, codec_ctx, frame, &frame_index, &packet, 1);
#endif

	gettimeofday(&tv_end, NULL);

	fclose(file);

#if RECOMPRESS
	if(codec_ctx->reCompressStreamFile){
		fclose(codec_ctx->reCompressStreamFile);
	}
	if(codec_ctx->deCompressStreamFile){
		fclose(codec_ctx->deCompressStreamFile);
	}
#else
	fclose(outfile);
#endif


	avcodec_close(codec_ctx);
	av_free(codec_ctx);
	av_parser_close(parser);
	av_frame_free(&frame);
	printf("Done\n");

	float time = (tv_end.tv_sec + (tv_end.tv_usec / 1000000.0)) - (tv_start.tv_sec + (tv_start.tv_usec / 1000000.0));
	float speed = (frame_index + 1) / time;
	printf("Decoding time: %.3fs, speed: %.1f FPS\n", time, speed);
}


// Test the broadway API
static FILE *foutput;

void broadwayOnPictureDecoded(u8 *buffer, u32 width, u32 height) {
	fwrite(buffer, width*height*3/2, 1, foutput);
}

static void broadway_decode(const char *filename, const char *outfilename)
{
	printf("Decode file '%s' to '%s'\n", filename, outfilename);

	FILE *finput = fopen(filename, "rb");
	if (!finput) {
		fprintf(stderr, "Could not open '%s'\n", filename);
		exit(1);
	}
	
	foutput = fopen(outfilename, "wb");
	if (!foutput) {
		fprintf(stderr, "Could not open '%s'\n", outfilename);
		exit(1);
	}
	
	fseek(finput, 0L, SEEK_END);
	u32 length = (u32)ftell(finput);
	rewind(finput);
	
	broadwayInit();
	u8* buffer = broadwayCreateStream(length);
	fread(buffer, sizeof(u8), length, finput);
	fclose(finput);
	
	broadwayParsePlayStream(length);
	
	broadwayExit();
	
	fclose(foutput);
}


#if RECOMPRESS
//获取文件大小
static unsigned long long int get_file_size(const char *filename){
	unsigned long long int size;
	struct stat64 st;
	stat64(filename, &st);
	size = st.st_size;

	return size;
}
//打印结果
static void printInfo(struct timeval start, struct timeval end, const char *in_file, const char *recon_stream, const char *cmd){
	unsigned long timer = 1e6 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
	printf("timer = %.4f s\n", (float)timer / 1e6);

	unsigned long long inSize = get_file_size(in_file);
	unsigned long long reconSize = get_file_size(recon_stream);
	float fSpeed = 0;
	float fRatio = 0;
	if(0 == strcmp(cmd, "-c") || 0 == strcmp(cmd, "-d")){
		if(0 == strcmp(cmd, "-c")){
			fSpeed = (double)inSize / (1024 * 1024) / ((double)timer / 1e6);
			fRatio = (double)(inSize) / reconSize;
		}else{
			fSpeed = (double)reconSize / (1024 * 1024) / ((double)timer / 1e6);
			fRatio = (double)(reconSize) / inSize;
		}
	}
	printf("inSize: %llu, outSize: %llu, ratio: %.4f, speed: %.2f MB/s\n", inSize, reconSize, fRatio, fSpeed);
}
#endif

int main(int argc, char* argv[])
{
#if RECOMPRESS
	const char *recompression_stream = NULL;
	const char *decompression_stream = NULL;
	const char *in_file = NULL;
	const char *cmd = NULL;

	if(argc == 4){
		cmd = argv[1];
		in_file = argv[2];
		if(0 == strcmp(cmd, "-c")){
			recompression_stream = argv[3];
		}else if(0 == strcmp(cmd, "-d")){
			decompression_stream = argv[3];
		}

		struct timeval start;
		struct timeval end;
		gettimeofday(&start, NULL);
		h264_video_decode(in_file, recompression_stream, decompression_stream);
		gettimeofday(&end, NULL);
		//打印结果
		printInfo(start, end, in_file, recompression_stream, cmd);
	} else {
		printf("Usage: %s <cmd>(-c/-d) <input_file> <output_file>\n", argv[0]);
	}
#else
	if (argc == 3) {
		h264_video_decode(argv[1], argv[2]);
	} else {
		printf("Usage: %s <input_file> <output_file>\n", argv[0]);
	}
#endif
	return 0;	
}
