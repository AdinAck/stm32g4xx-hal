#![deny(missing_docs)]

use crate::{rcc::Rcc, stm32::CORDIC};
use core::marker::PhantomData;
use fixed::{
    traits::Fixed,
    types::{I1F15, I1F31},
};

/// Extension trait for constraining the CORDIC peripheral.
pub trait Ext {
    /// Constrain the CORDIC peripheral.
    fn constrain(self, rcc: &mut Rcc) -> CordicReset;
}

impl Ext for CORDIC {
    #[inline]
    fn constrain(self, rcc: &mut Rcc) -> CordicReset {
        rcc.rb.ahb1enr.modify(|_, w| w.cordicen().set_bit());

        // lock until enabled
        rcc.rb.ahb1enr.read().cordicen();

        // SAFETY: the resource is assumed to be
        // in the "reset" configuration.
        unsafe { Cordic::wrap(self) }
    }
}

/// Trait for newtypes to represent CORDIC argument or result data.
pub trait DataType {
    /// The underlying fixed-point data type.
    type Fixed: Fixed;
}

/// Traits and structures related to argument type-states.
pub mod arg_type {
    use super::*;

    /// Trait for argument type-states.
    pub trait State {
        /// Bit representation of the argument type.
        const BITS: bool;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: &mut crate::stm32::cordic::csr::W);
    }

    /// q1.31 fixed point format.
    pub struct Q31;
    /// q1.15 fixed point format.
    pub struct Q15;

    impl DataType for Q31 {
        type Fixed = I1F31;
    }

    impl DataType for Q15 {
        type Fixed = I1F15;
    }

    macro_rules! impls {
        ( $( ($NAME:ident, $SIZE:ident, $BITS:expr) $(,)?)+ ) => {
            $(
                impl State for $NAME {
                    const BITS: bool = $BITS;

                    #[inline]
                    fn set(w: &mut crate::stm32::cordic::csr::W) {
                        w.argsize().$SIZE();
                    }
                }
            )+
        };
    }

    impls! {
        (Q31, bits32, false),
        (Q15, bits16, true),
    }
}

/// Traits and structures related to result type-states.
pub mod res_type {
    use super::*;

    /// Trait for result type-states.
    pub trait State {
        /// Bit representation of the result type.
        const BITS: bool;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: &mut crate::stm32::cordic::csr::W);
    }

    /// q1.31 fixed point format.
    pub struct Q31;
    /// q1.15 fixed point format.
    pub struct Q15;

    impl DataType for Q31 {
        type Fixed = I1F31;
    }

    impl DataType for Q15 {
        type Fixed = I1F15;
    }

    macro_rules! impls {
        ( $( ($NAME:ident, $SIZE:ident, $BITS:expr) $(,)?)+ ) => {
            $(
                impl State for $NAME {
                    const BITS: bool = $BITS;

                    #[inline]
                    fn set(w: &mut crate::stm32::cordic::csr::W) {
                        w.ressize().$SIZE();
                    }
                }
            )+
        };
    }

    impls! {
        (Q31, bits32, false),
        (Q15, bits16, true),
    }
}

/// Traits and structures related to function type-states.
pub mod func {
    use super::*;

    /// Traits and structures related to function argument count type-states.
    pub mod nargs {
        use super::*;

        /// Trait for function argument type-states.
        pub trait State<Arg>
        where
            Arg: arg_type::State,
        {
            /// Configure the resource to be represented
            /// by this type-state.
            fn set(w: &mut crate::stm32::cordic::csr::W);
        }

        /// One (primary) function argument.
        pub struct One;
        /// Two (primary and secondary) function arguements.
        pub struct Two;

        impl<Arg> State<Arg> for One
        where
            Arg: arg_type::State,
        {
            fn set(w: &mut crate::stm32::cordic::csr::W) {
                w.nargs().num1();
            }
        }

        impl<Arg> State<Arg> for Two
        where
            Arg: arg_type::State,
        {
            fn set(w: &mut crate::stm32::cordic::csr::W) {
                w.nargs().bit(!Arg::BITS);
            }
        }
    }

    /// Traits and structures related to function result count type-states.
    pub mod nres {
        use super::*;

        /// Trait for function result type-states.
        pub trait State<Res>
        where
            Res: res_type::State,
        {
            /// Configure the resource to be represented
            /// by this type-state.
            fn set(w: &mut crate::stm32::cordic::csr::W);
        }

        /// One (primary) function result.
        pub struct One;
        /// Two (primary and secondary) function results.
        pub struct Two;

        impl<Res> State<Res> for One
        where
            Res: res_type::State,
        {
            fn set(w: &mut crate::stm32::cordic::csr::W) {
                w.nargs().num1();
            }
        }

        impl<Res> State<Res> for Two
        where
            Res: res_type::State,
        {
            fn set(w: &mut crate::stm32::cordic::csr::W) {
                w.nres().bit(!Res::BITS);
            }
        }
    }

    /// Traits and structures related to function scale type-states.
    pub mod scale {
        /// Trait for function scale type-states.
        pub trait State {
            /// Bit representation of the scale.
            const BITS: u8;

            /// Configure the resource to be represented
            /// by this type-state.
            fn set(w: &mut crate::stm32::cordic::csr::W);
        }

        /// Scale of 0.
        pub struct N0;
        /// Scale of 1.
        pub struct N1;
        /// Scale of 2.
        pub struct N2;
        /// Scale of 3.
        pub struct N3;
        /// Scale of 4.
        pub struct N4;
        /// Scale of 5.
        pub struct N5;
        /// Scale of 6.
        pub struct N6;
        /// Scale of 7.
        pub struct N7;

        macro_rules! impls {
            ( $( ($NAME:ident, $BITS:expr) $(,)? )+ ) => {
                $(
                    impl State for $NAME {
                        const BITS: u8 = $BITS;

                        #[inline]
                        fn set(w: &mut crate::stm32::cordic::csr::W) {
                            w.scale().bits(<Self as State>::BITS);
                        }
                    }
                )+
            };
        }

        impls! {
            (N0, 0),
            (N1, 1),
            (N2, 2),
            (N3, 3),
            (N4, 4),
            (N5, 5),
            (N6, 6),
            (N7, 7),
        }
    }

    /// Trait for function type-states.
    pub trait State<Arg, Res>
    where
        Arg: arg_type::State,
        Res: res_type::State,
    {
        /// The number of arguments required by this function.
        type Args: nargs::State<Arg>;
        /// The number of arguments produced by this function.
        type Results: nres::State<Res>;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: &mut crate::stm32::cordic::csr::W);
    }

    // function types with argument count encoded

    /// Cosine of an angle theta divided by pi.
    pub struct Cos;
    /// Sine of an angle theta divided by pi.
    pub struct Sin;
    /// Sine (primary) and cosine (secondary) of an angle theta divided by pi.
    pub struct SinCos;
    /// Modulus (secondary) multiplied by cosine of an angle theta divided by pi (primary).
    pub struct CosM;
    /// Modulus (secondary) multiplied by sine of an angle theta divided by pi (primary).
    pub struct SinM;
    /// Modulus (secondary) multiplied by sine (primary) and cosine (secondary) of an angle theta divided by pi (primary).
    pub struct SinCosM;
    /// Arctangent of x (primary) and y (secondary).
    pub struct ATan2;
    /// Magnitude of x (primary) and y (secondary).
    pub struct Magnitude;
    /// Arctangent (primary) and magnitude (secondary) of x (primary) and y (secondary).
    pub struct ATan2Magnitude;
    /// Arctangent of x.
    ///
    /// This function can be scaled by 0-7.
    pub struct ATan<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }
    /// Hyperbolic cosine of x.
    pub struct CosH;
    /// Hyperbolic sine of x.
    pub struct SinH;
    /// Hyperbolic sine (primary) and cosine (secondary) of x.
    pub struct SinHCosH;
    /// Hyperbolic arctangent of x.
    pub struct ATanH;
    /// Natural logarithm of x.
    ///
    /// This function can be scaled by 1-4.
    pub struct Ln<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }
    /// Square root of x.
    ///
    /// This function can be scaled by 0-2.
    pub struct Sqrt<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }

    macro_rules! impls {
        // root / config
        ( $( ($NAME:ident < $SCALE:ty >, $FUNC:ident, nargs::$NARGS:ident, nres::$NRES:ident, start( $($START_PARAM:ident),+ )) $(,)?)+ ) => {
            $(
                impl<Arg, Res> State<Arg, Res> for $NAME
                where
                    Arg: arg_type::State,
                    Res: res_type::State,
                {
                    type Args = nargs::$NARGS;
                    type Results = nres::$NRES;

                    #[inline]
                    fn set(w: &mut crate::stm32::cordic::csr::W) {
                        <nargs::$NARGS as nargs::State<Arg>>::set(w);
                        <nres::$NRES as nres::State<Res>>::set(w);
                        w.func().$FUNC();
                    }
                }

                impls!($NAME, nargs::$NARGS, start( $($START_PARAM),+ ));
                impls!($NAME, nres::$NRES);
            )+
        };

        // impl start for one arg
        ($NAME:ty, nargs::One, start( $PRIMARY:ident )) => {
            // arg_type: Q31
            // nargs: 1
            impl<Res, Prec> Cordic<arg_type::Q31, Res, $NAME, Prec>
            where
                Res: res_type::State,
                Prec: prec::State,
            {
                #[doc = "Start evaluating the configured function"]
                #[doc = "with the provided inputs."]
                #[inline]
                pub fn start(&mut self, $PRIMARY: <arg_type::Q31 as DataType>::Fixed) {
                    self.rb
                        .wdata
                        .write(|w| w.arg().bits($PRIMARY.to_bits() as _));
                }
            }

            // arg_type: Q15
            // nargs: 1
            impl<Res, Prec> Cordic<arg_type::Q15, Res, $NAME, Prec>
            where
                Res: res_type::State,
                Prec: prec::State,
            {
                #[doc = "Start evaluating the configured function"]
                #[doc = "with the provided inputs."]
                #[inline]
                pub fn start(&mut self, $PRIMARY: <arg_type::Q15 as DataType>::Fixed) {
                    // $RM0440 17.4.2
                    // since we are only using the lower half of the register,
                    // the CORDIC **will** read the upper half if the function
                    // accepts two arguments, so we fill it with +1 as per the
                    // stated default.
                    let reg = ($PRIMARY.to_bits() as u16 as u32) | (0x7fff << 16);

                    self.rb.wdata.write(|w| w.arg().bits(reg));
                }
            }
        };

        // impl start for two args
        ($NAME:ty, nargs::Two, start( $PRIMARY:ident, $SECONDARY:ident )) => {
            // arg_type: Q31
            // nargs: 2
            impl<Res, Prec> Cordic<arg_type::Q31, Res, $NAME, Prec>
            where
                Res: res_type::State,
                Prec: prec::State,
            {
                #[doc = "Start evaluating the configured function"]
                #[doc = "with the provided inputs."]
                #[inline]
                pub fn start(
                    &mut self,
                    $PRIMARY: <arg_type::Q31 as DataType>::Fixed,
                    $SECONDARY: <arg_type::Q31 as DataType>::Fixed
                ) {
                    self.rb
                        .wdata
                        .write(|w| w.arg().bits($PRIMARY.to_bits() as _));
                    self.rb
                        .wdata
                        .write(|w| w.arg().bits($SECONDARY.to_bits() as _));
                }
            }

            // arg_type: Q15
            // nargs: 2
            impl<Res, Prec> Cordic<arg_type::Q15, Res, $NAME, Prec>
            where
                Res: res_type::State,
                Prec: prec::State,
            {
                #[doc = "Start evaluating the configured function"]
                #[doc = "with the provided inputs."]
                #[inline]
                pub fn start(
                    &mut self,
                    $PRIMARY: <arg_type::Q15 as DataType>::Fixed,
                    $SECONDARY: <arg_type::Q15 as DataType>::Fixed
                ) {
                    // $RM0440 17.4.2
                    let reg = ($PRIMARY.to_bits() as u16 as u32) | (($SECONDARY.to_bits() as u16 as u32) << 16);

                    self.rb.wdata.write(|w| w.arg().bits(reg));
                }
            }
        };

        // impl result for one result
        ($NAME:ty, nres::One) => {
            // res_type: Q31
            // nres: 1
            impl<Arg, Prec> Cordic<Arg, res_type::Q31, $NAME, Prec>
            where
                Arg: arg_type::State,
                Prec: prec::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is ongoing.*"]
                #[inline]
                pub fn result(&mut self) -> <res_type::Q31 as DataType>::Fixed {
                    <res_type::Q31 as DataType>::Fixed::from_bits(
                        self.rb.rdata.read().res().bits() as _
                    )
                }
            }

            // res_type: Q15
            // nres: 1
            impl<Arg, Prec> Cordic<Arg, res_type::Q15, $NAME, Prec>
            where
                Arg: arg_type::State,
                Prec: prec::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is ongoing.*"]
                #[inline]
                pub fn result(&mut self) -> <res_type::Q15 as DataType>::Fixed {
                    <res_type::Q15 as DataType>::Fixed::from_bits(
                        self.rb.rdata.read().res().bits() as _,
                    )
                }
            }
        };

        // impl result for two results
        ($NAME:ty, nres::Two) => {
            // res_type: Q31
            // nres: 2
            impl<Arg, Prec> Cordic<Arg, res_type::Q31, $NAME, Prec>
            where
                Arg: arg_type::State,
                Prec: prec::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is ongoing.*"]
                #[inline]
                pub fn result(
                    &mut self,
                ) -> (
                    <res_type::Q31 as DataType>::Fixed,
                    <res_type::Q31 as DataType>::Fixed,
                ) {
                    (
                        <res_type::Q31 as DataType>::Fixed::from_bits(self.rb.rdata.read().res().bits() as _),
                        <res_type::Q31 as DataType>::Fixed::from_bits(self.rb.rdata.read().res().bits() as _),
                    )
                }
            }

            // res_type: Q15
            // nres: 2
            impl<Arg, Prec> Cordic<Arg, res_type::Q15, $NAME, Prec>
            where
                Arg: arg_type::State,
                Prec: prec::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is ongoing.*"]
                #[inline]
                pub fn result(
                    &mut self,
                ) -> (
                    <res_type::Q15 as DataType>::Fixed,
                    <res_type::Q15 as DataType>::Fixed,
                ) {
                    let reg = self.rb.rdata.read().res().bits();

                    // $RM0440 17.4.3
                    (
                        <res_type::Q15 as DataType>::Fixed::from_bits((reg & 0xffff) as _),
                        <res_type::Q15 as DataType>::Fixed::from_bits((reg >> 16) as _),
                    )
                }
            }
        };
    }

    macro_rules! impls_multi_scale {
        // root / config
        ( $( ($NAME:ident < $( $SCALE:ty  $(,)? )+ >, $FUNC:ident, nargs::$NARGS:ident, nres::$NRES:ident, start $START_PARAM:tt ) $(,)?)+ ) => {
            $(
                $(
                    impl<Arg, Res> State<Arg, Res> for $NAME<$SCALE>
                    where
                        Arg: arg_type::State,
                        Res: res_type::State,
                    {
                        type Args = nargs::$NARGS;
                        type Results = nres::$NRES;

                        #[inline]
                        fn set(w: &mut crate::stm32::cordic::csr::W) {
                            <nargs::$NARGS as nargs::State<Arg>>::set(w);
                            <nres::$NRES as nres::State<Res>>::set(w);
                            w.func().$FUNC();
                        }
                    }

                    impls!($NAME<$SCALE>, nargs::$NARGS, start $START_PARAM );
                    impls!($NAME<$SCALE>, nres::$NRES);
                )+
            )+
        };
    }

    impls! {
        (Cos<scale::N0>, cosine, nargs::One, nres::One, start(angle)),
        (Sin<scale::N0>, sine, nargs::One, nres::One, start(angle)),
        (SinCos<scale::N0>, sine, nargs::One, nres::Two, start(angle)),
        (CosM<scale::N0>, cosine, nargs::Two, nres::One, start(angle, modulus)),
        (SinM<scale::N0>, sine, nargs::Two, nres::One, start(angle, modulus)),
        (SinCosM<scale::N0>, sine, nargs::Two, nres::Two, start(angle, modulus)),
        (ATan2<scale::N0>, phase, nargs::Two, nres::One, start(x, y)),
        (Magnitude<scale::N0>, modulus, nargs::Two, nres::One, start(x, y)),
        (ATan2Magnitude<scale::N0>, phase, nargs::Two, nres::Two, start(x, y)),
        (CosH<scale::N1>, hyperbolic_cosine, nargs::One, nres::One, start(x)),
        (SinH<scale::N1>, hyperbolic_sine, nargs::One, nres::One, start(x)),
        (SinHCosH<scale::N1>, hyperbolic_cosine, nargs::One, nres::Two, start(x)),
        (ATanH<scale::N1>, arctanh, nargs::One, nres::One, start(x)),
    }

    impls_multi_scale! {
        (ATan<scale::N0, scale::N1, scale::N2, scale::N3, scale::N4, scale::N5, scale::N6, scale::N7>, arctangent, nargs::One, nres::One, start(x)),
        (Ln<scale::N1, scale::N2, scale::N3, scale::N4>, natural_logarithm, nargs::One, nres::One, start(x)),
        (Sqrt<scale::N0, scale::N1, scale::N2>, square_root, nargs::One, nres::One, start(x)),
    }
}

/// Traits and structures related to precision type-states.
pub mod prec {
    /// Trait for precision type-states.
    pub trait State {
        /// Bit representation of the precision.
        const BITS: u8;

        /// Configure the resource to be represented
        /// by this type-state.
        fn set(w: &mut crate::stm32::cordic::csr::W);
    }

    /// 4 iterations.
    pub struct P4;
    /// 8 iterations.
    pub struct P8;
    /// 12 iterations.
    pub struct P12;
    /// 16 iterations.
    pub struct P16;
    /// 20 iterations.
    pub struct P20;
    /// 24 iterations.
    pub struct P24;
    /// 28 iterations.
    pub struct P28;
    /// 32 iterations.
    pub struct P32;
    /// 36 iterations.
    pub struct P36;
    /// 40 iterations.
    pub struct P40;
    /// 44 iterations.
    pub struct P44;
    /// 48 iterations.
    pub struct P48;
    /// 52 iterations.
    pub struct P52;
    /// 56 iterations.
    pub struct P56;
    /// 60 iterations.
    pub struct P60;

    macro_rules! impls {
        ( $( ($NAME:ident, $BITS:expr) $(,)? )+ ) => {
            $(
                impl State for $NAME {
                    const BITS: u8 = $BITS;

                    #[inline]
                    fn set(w: &mut crate::stm32::cordic::csr::W) {
                        // SAFETY: reliant valid type-state
                        // implementations.
                        unsafe { w.precision().bits(<Self as State>::BITS) };
                    }
                }
            )+
        };
    }

    impls! {
        (P4, 1),
        (P8, 2),
        (P12, 3),
        (P16, 4),
        (P20, 5),
        (P24, 6),
        (P28, 7),
        (P32, 8),
        (P36, 9),
        (P40, 10),
        (P44, 11),
        (P48, 12),
        (P52, 13),
        (P56, 14),
        (P60, 15),
    }
}

/// Cordic co-processor interface.
pub struct Cordic<Arg, Res, Func, Prec>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State<Arg, Res>,
    Prec: prec::State,
{
    rb: CORDIC,
    _arg_size: PhantomData<Arg>,
    _res_size: PhantomData<Res>,
    _func: PhantomData<Func>,
    _prec: PhantomData<Prec>,
}

// root impl
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State<Arg, Res>,
    Prec: prec::State,
{
    /// Configure the resource as dictated by the resulting
    /// type-states. The produced binding represents
    /// a frozen configuration, since it is represented
    /// by types. A new binding will need to be made --
    /// and the old binding invalidated -- in order to change
    /// the configuration.
    ///
    /// *Note: The configuration is inferred from context because
    /// it is represented by generic type-states.*
    pub fn freeze<NewArg, NewRes, NewFunc, NewPrec>(
        self,
    ) -> Cordic<NewArg, NewRes, NewFunc, NewPrec>
    where
        NewArg: arg_type::State,
        NewRes: res_type::State,
        NewFunc: func::State<NewArg, NewRes>,
        NewPrec: prec::State,
    {
        self.rb.csr.write(|w| {
            NewArg::set(w);
            NewRes::set(w);
            NewFunc::set(w);
            NewPrec::set(w);

            w
        });

        // SAFETY: the resource has been configured
        // to represent the new type-states.
        unsafe { Cordic::wrap(self.rb) }
    }

    /// Determine whether a result is pending or not.
    #[inline]
    pub fn is_ready(&self) -> bool {
        self.rb.csr.read().rrdy().bit_is_set()
    }

    /// Wrap the resource as a noop.
    ///
    /// # Safety
    ///
    /// If the resource configuration and
    /// type-states are incongruent, the invariance
    /// is broken and actions may exhibit
    /// undefined behavior.
    pub const unsafe fn wrap(rb: CORDIC) -> Self {
        Self {
            rb,
            _arg_size: PhantomData,
            _res_size: PhantomData,
            _func: PhantomData,
            _prec: PhantomData,
        }
    }
}

/// $RM0440 17.4.1
pub type CordicReset = Cordic<arg_type::Q31, res_type::Q31, func::Cos, prec::P20>;

impl<Arg, Res, Func, Prec> proto::IntoReset for Cordic<Arg, Res, Func, Prec>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State<Arg, Res>,
    Prec: prec::State,
{
    type Reset = CordicReset;

    #[inline]
    fn into_reset(self) -> Self::Reset {
        self.freeze()
    }
}

// listen
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State<Arg, Res>,
    Prec: prec::State,
{
    /// Enable the result ready interrupt.
    #[inline]
    pub fn listen(&mut self) {
        self.rb.csr.modify(|_, w| w.ien().set_bit());
    }

    /// Disable the result ready interrupt.
    #[inline]
    pub fn unlisten(&mut self) {
        self.rb.csr.modify(|_, w| w.ien().clear_bit());
    }
}

// release
impl<Arg, Res, Func, Prec> Cordic<Arg, Res, Func, Prec>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State<Arg, Res>,
    Prec: prec::State,
{
    /// Release the CORDIC resource binding as a noop.
    ///
    /// # Safety:
    ///
    /// The CORDIC peripheral is not reset.
    #[inline]
    pub unsafe fn release(self) -> CORDIC {
        self.rb
    }

    /// Release the CORDIC resource binding after reset.
    #[inline]
    pub fn release_and_reset(self, rcc: &mut Rcc) -> CORDIC {
        let reset: CordicReset = self.freeze();

        rcc.rb.ahb1enr.modify(|_, w| w.cordicen().clear_bit());

        // SAFETY: the resource has been reset
        unsafe { reset.release() }
    }
}
