#include "Colour.h"
#include "Maths.h"

const Colour Colour::RED = { 1, 0, 0 };
const Colour Colour::GREEN = { 0, 1, 0 };
const Colour Colour::BLUE = { 0, 0, 1 };

const Colour Colour::CYAN = { 0, 1, 1 };
const Colour Colour::MAGENTA = { 1, 0, 1 };
const Colour Colour::YELLOW = { 1, 1, 0 };

const Colour Colour::WHITE = { 1, 1, 1 };
const Colour Colour::GREY = { 0.5f, 0.5f, 0.5f };
const Colour Colour::BLACK = { 0, 0, 0 };

const Colour Colour::ORANGE = { 1.0f, 0.67f, 0.0f };

Colour Colour::Multiply(float factor) const
{
	return Colour(r * factor, g * factor, b * factor);
}

Colour Colour::Desaturate(float factor) const
{
	return this->Invert().Multiply(1.0f - factor).Invert();
}

Colour Colour::Invert() const
{
	return Colour(1.0f - r, 1.0f - g, 1.0f - b);
}

Colour Colour::Mix(Colour base, Colour overlay, float alpha)
{
	return Colour(
		Lerp(base.r, overlay.r, alpha), 
		Lerp(base.g, overlay.g, alpha), 
		Lerp(base.b, overlay.b, alpha)
	);
}