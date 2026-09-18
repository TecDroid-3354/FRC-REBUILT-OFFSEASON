package frc.tecdroid3354.utils.controlProfiles

/** Wrapper for polynomials of any degree. Use [Polynomials.of] when instantiating, as it handles parameter failures.
 * When evaluating, use polynomial.getOrThrow().evaluate; there will be an [IllegalArgumentException] if the polynomial
 * wasn't instantiated properly.
 * @param degree The polynomial degree (e.g. degree = 3 -> x^3 + x^2 + x^1 + x^0)
 * @param coefficients The list of coefficients from higher to lower degree. This must contain [degree] + 1 elements */
class Polynomials private constructor(
    val degree: Int,
    val coefficients: DoubleArray
) {
    /** Takes the input parameter of the polynomial and evaluates using Horner's Method */
    fun evaluate(x: Double): Double {
        return coefficients.fold(0.0) { acc, coeff -> acc * x + coeff }
    }

    companion object {
        /** Returns Kotlin's Result type, capturing errors without throwing exceptions, that will be triggered when accessing */
        fun of(degree: Int, vararg coefficients: Double): Result<Polynomials> {
            return when {
                degree < 0 ->
                    Result.failure(IllegalArgumentException("At Polynomial Initialization: Degree must be non-negative."))
                coefficients.size != degree + 1 ->
                    Result.failure(IllegalArgumentException("At Polynomial Initialization: Degree $degree requires exactly ${degree + 1} coefficients."))
                else ->
                    Result.success(Polynomials(degree, coefficients))
            }
        }
    }
}