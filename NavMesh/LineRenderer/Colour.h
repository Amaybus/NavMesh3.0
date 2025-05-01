#pragma once

struct Colour
{
	float r = 1.0f, g = 1.0f, b = 1.0f;
	Colour() {}
	Colour(float red, float green, float blue) : r(red), g(green), b(blue) {}

	Colour Multiply(float factor) const;
	Colour Desaturate(float factor) const;
	Colour Invert() const;

	Colour Darken() const { return Multiply(0.5f); }
	Colour Lighten() const { return Desaturate(0.5f); }

	static const Colour RED;
	static const Colour GREEN;
	static const Colour BLUE;

	static const Colour CYAN;
	static const Colour MAGENTA;
	static const Colour YELLOW;

	static const Colour WHITE;
	static const Colour BLACK;
	static const Colour GREY;

	static const Colour ORANGE;


	static Colour Mix(Colour base, Colour overlay, float alpha);
};
