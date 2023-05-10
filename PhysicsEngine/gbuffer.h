#pragma once


class GBuffer
{
public:

    enum GBUFFER_TEXTURE_TYPE {
        GBUFFER_TEXTURE_TYPE_POSITION,
        GBUFFER_TEXTURE_TYPE_DIFFUSE,
        GBUFFER_TEXTURE_TYPE_NORMAL,
        GBUFFER_TEXTURE_TYPE_TEXCOORD,
        GBUFFER_NUM_TEXTURES
    };

    GBuffer();

    ~GBuffer();

    bool Init(unsigned int WindowWidth, unsigned int WindowHeight);
	void BindFBO();

	void UnbindFBO();
    void BindTexture(const int unit, const int programId, const std::string& name, const int textureId);

    void UnbindTexture(const int unit);

private:

    unsigned int fbo;
    unsigned int textures[GBUFFER_NUM_TEXTURES];
    unsigned int depthTexture;
};