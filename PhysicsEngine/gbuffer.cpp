#include <glbinding/gl/gl.h>
#include <glbinding/Binding.h>
using namespace gl;

#include "gbuffer.h"

#define ARRAY_SIZE_IN_ELEMENTS(m) sizeof(m)/sizeof(m[0])

GBuffer::GBuffer()
{
}

GBuffer::~GBuffer()
{
}

bool GBuffer::Init(unsigned int WindowWidth, unsigned int WindowHeight)
{

    glGenFramebuffersEXT(1, &fbo);
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);

    // Create a render buffer, and attach it to FBO's depth attachment
    unsigned int depthBuffer;
    glGenRenderbuffersEXT(1, &depthBuffer);
    glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, depthBuffer);
    glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT,
        WindowWidth, WindowHeight);
    glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT,
        GL_RENDERBUFFER_EXT, depthBuffer);

    // Create a texture and attach FBO's color 0 attachment.  The
    // GL_RGBA32F and GL_RGBA constants set this texture to be 32 bit
    // floats for each of the 4 components.  Many other choices are
    // possible.

    for (int i = 0; i < 4; ++i)
    {
        glGenTextures(1, &textures[i]);
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, (int)GL_RGBA32F, WindowWidth, WindowHeight, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0);


        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, (int)GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, (int)GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, (int)GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (int)GL_LINEAR);

        glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, (gl::GLenum)((int)GL_COLOR_ATTACHMENT0_EXT + i),
            GL_TEXTURE_2D, textures[i], 0);
    }
    // Check for completeness/correctness
    int status = (int)glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
    if (status != int(GL_FRAMEBUFFER_COMPLETE_EXT))
        printf("FBO Error: %d\n", status);

    // Unbind the fbo until it's ready to be used
    glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
    return true;

    /*
    // Create the FBO
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);

    // Create the gbuffer textures
    glGenTextures(ARRAY_SIZE_IN_ELEMENTS(textures), textures);
    glGenTextures(1, &depthTexture);

    for (unsigned int i = 0; i < ARRAY_SIZE_IN_ELEMENTS(textures); i++) {
        glBindTexture(GL_TEXTURE_2D, textures[i]);
        glTexImage2D(GL_TEXTURE_2D, 0, (GLint)GL_RGBA32F, WindowWidth, WindowHeight, 0, GL_RGBA, GL_FLOAT, NULL);
        glFramebufferTexture2D(GL_FRAMEBUFFER, (gl::GLenum)((int)GL_COLOR_ATTACHMENT0 + i), GL_TEXTURE_2D, textures[i], 0);
    }

    // depth
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, (GLint)GL_DEPTH_COMPONENT32F, WindowWidth, WindowHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
        NULL);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTexture, 0);

    

    GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

    if (Status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", Status);
        return false;
    }

    // restore default FBO
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    return true;
    */
}

void GBuffer::BindFBO()
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo);
}

void GBuffer::UnbindFBO()
{
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}

void GBuffer::BindTexture(const int unit, const int programId, const std::string& name, const int textureId)
{
    glActiveTexture((gl::GLenum)((int)GL_TEXTURE0 + unit));
    glBindTexture(GL_TEXTURE_2D, textures[textureId]);
    int loc = glGetUniformLocation(programId, name.c_str());
    glUniform1i(loc, unit);
}

void GBuffer::UnbindTexture(const int unit)
{
    glActiveTexture((gl::GLenum)((int)GL_TEXTURE0 + unit));
    glBindTexture(GL_TEXTURE_2D, 0);
}

