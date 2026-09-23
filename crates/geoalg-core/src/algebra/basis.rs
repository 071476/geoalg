//! Base ortonormal del álgebra: representación de las láminas básicas.
//!
//! Una lámina se identifica con una máscara de bits donde el bit `i`
//! representa al vector de base `e_i`:
//!
//!   bit 0 -> e0 (vector nulo del infinito en PGA)
//!   bit 1 -> e1, bit 2 -> e2, bit 3 -> e3
//!
//! El índice canónico de una lámina es su propia máscara (0..=15), lo que da
//! un orden total sobre las 16 láminas de R(3,0,1).

/// Dimensión del espacio vectorial subyacente (R^4 en PGA 3D).
pub const DIM: usize = 4;

/// Número de láminas básicas (2^DIM).
pub const NBLADES: usize = 1 << DIM;

/// Máscara de bits que identifica una lámina básica.
pub type Blade = u8;

/// Vectores de base canónicos.
pub const E0: Blade = 1 << 0;
pub const E1: Blade = 1 << 1;
pub const E2: Blade = 1 << 2;
pub const E3: Blade = 1 << 3;

/// Pseudoscalar del álgebra (e0123).
pub const I: Blade = 0b1111;

/// Grado de una lámina (número de vectores de base que la componen).
pub const fn grade(b: Blade) -> usize {
    // Contamos bits a mano para poder usarse en contexto `const`.
    let mut n = 0;
    let mut x = b;
    while x != 0 {
        n += (x & 1) as usize;
        x >>= 1;
    }
    n
}

/// Nombres canónicos de las 16 láminas, indexados por máscara.
pub const BLADE_NAMES: [&str; NBLADES] = [
    "1", "e0", "e1", "e01", "e2", "e02", "e12", "e012",
    "e3", "e03", "e13", "e013", "e23", "e023", "e123", "e0123",
];