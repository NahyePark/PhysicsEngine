///////////////////////////////////////////////////////////////////////
// A slight encapsulation of a Frame Buffer Object (i'e' Render
// Target) and its associated texture.  When the FBO is "Bound", the
// output of the graphics pipeline is captured into the texture.  When
// it is "Unbound", the texture is available for use as any normal
// texture.
////////////////////////////////////////////////////////////////////////

class FBO {
public:
    unsigned int fboID;
    unsigned int textureID;
    int width, height;  // Size of the texture.

    void CreateFBO(const int w, const int h);
    
    // Bind this FBO to receive the output of the graphics pipeline.
    void BindFBO();
    
    // Unbind this FBO from the graphics pipeline;  graphics goes to screen by default.
    void UnbindFBO();

    // Bind this FBO's texture to a texture unit.
    void BindTexture(const int unit, const int programId, const std::string& name);

    // Unbind this FBO's texture from a texture unit.
    void UnbindTexture(const int unit);
};
