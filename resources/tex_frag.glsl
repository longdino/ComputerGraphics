#version 120

uniform sampler2D colorTexture;
varying vec2 vTex;

void main()
{
	vec4 c = texture2D(colorTexture, vTex);
	gl_FragColor = vec4(c.rgb, 1.0);
}
