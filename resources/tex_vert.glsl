#version 120

attribute vec4 aPos;
attribute vec2 aTex;
uniform mat4 P;
uniform mat4 MV;
varying vec2 vTex;

void main()
{
	gl_Position = P * MV * aPos;
	vTex = aTex;
}
