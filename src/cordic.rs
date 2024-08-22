use crate::{rcc::Rcc, stm32::CORDIC};
use core::marker::PhantomData;
use fixed::{
    traits::Fixed,
    types::{I1F15, I1F31},
};

// extension trait for resource
pub trait Ext {
    fn constrain(self, rcc: &mut Rcc) -> CordicReset;
}

impl Ext for CORDIC {
    #[inline]
    fn constrain(self, rcc: &mut Rcc) -> CordicReset {
        rcc.rb.ahb1enr.modify(|_, w| w.cordicen().set_bit());
        // SAFETY: we assume the resource
        // is already in a reset state.
        unsafe { CordicReset::wrap(self) }
    }
}

pub trait DataType {
    type Fixed: Fixed;
}

pub mod arg_type {
    use super::*;

    pub trait State: proto::TypeState<Binding = CORDIC> {}

    pub struct Q31;
    pub struct Q15;

    impl DataType for Q31 {
        type Fixed = I1F31;
    }

    impl DataType for Q15 {
        type Fixed = I1F15;
    }

    macro_rules! impls {
        ( $( ($NAME:ident, $FUNC:ident) $(,)?)+ ) => {
            $(
                impl proto::TypeState for $NAME {
                    type Binding = CORDIC;

                    #[inline]
                    fn set(rb: &mut Self::Binding) {
                        rb.csr.modify(|_, w| w.argsize().$FUNC());
                    }
                }

                impl State for $NAME {}
            )+
        };
    }

    impls! {
        (Q31, bits32),
        (Q15, bits16),
    }

    impl<Arg, Res, Func> Cordic<Arg, Res, Func>
    where
        Arg: arg_type::State,
        Res: res_type::State,
        Func: func::State,
    {
        #[inline]
        pub fn arg_type<NewArg>(mut self) -> Cordic<NewArg, Res, Func>
        where
            NewArg: arg_type::State,
        {
            NewArg::set(&mut self.rb);

            // SAFETY: the resource was configured
            // such that it is represented by the new
            // type-states
            unsafe { Cordic::wrap(self.rb) }
        }
    }
}

pub mod res_type {
    use super::*;

    pub trait State: proto::TypeState<Binding = CORDIC> {}

    pub struct Q31;
    pub struct Q15;

    impl DataType for Q31 {
        type Fixed = I1F31;
    }

    impl DataType for Q15 {
        type Fixed = I1F15;
    }

    macro_rules! impls {
        ( $( ($NAME:ident, $FUNC:ident) $(,)?)+ ) => {
            $(
                impl proto::TypeState for $NAME {
                    type Binding = CORDIC;

                    #[inline]
                    fn set(rb: &mut Self::Binding) {
                        rb.csr.modify(|_, w| w.ressize().$FUNC());
                    }
                }

                impl State for $NAME {}
            )+
        };
    }

    impls! {
        (Q31, bits32),
        (Q15, bits16),
    }

    impl<Arg, Res, Func> Cordic<Arg, Res, Func>
    where
        Arg: arg_type::State,
        Res: res_type::State,
        Func: func::State,
    {
        #[inline]
        pub fn res_type<NewRes>(mut self) -> Cordic<Arg, NewRes, Func>
        where
            NewRes: res_type::State,
        {
            NewRes::set(&mut self.rb);

            // SAFETY: the resource was configured
            // such that it is represented by the new
            // type-states
            unsafe { Cordic::wrap(self.rb) }
        }
    }
}

pub mod func {
    use super::*;

    pub mod nargs {
        use super::*;

        pub trait State: proto::TypeState<Binding = CORDIC> {}

        pub struct One;
        pub struct Two;

        macro_rules! impls {
            ( $( ($NAME:ident, $NARGS:ident) $(,)?)+ ) => {
                $(
                    impl proto::TypeState for $NAME {
                        type Binding = CORDIC;

                        #[inline]
                        fn set(rb: &mut Self::Binding) {
                            rb.csr.modify(|_, w| w.nargs().$NARGS());
                        }
                    }

                    impl State for $NAME {}
                )+
            };
        }

        impls! {
            (One, num1),
            (Two, num2),
        }
    }

    pub mod nres {
        use super::*;

        pub trait State: proto::TypeState<Binding = CORDIC> {}

        pub struct One;
        pub struct Two;

        macro_rules! impls {
            ( $( ($NAME:ident, $NRES:ident) $(,)?)+ ) => {
                $(
                    impl proto::TypeState for $NAME {
                        type Binding = CORDIC;

                        #[inline]
                        fn set(rb: &mut Self::Binding) {
                            rb.csr.modify(|_, w| w.nres().$NRES());
                        }
                    }

                    impl State for $NAME {}
                )+
            };
        }

        impls! {
            (One, num1),
            (Two, num2),
        }
    }

    pub mod scale {
        use super::*;

        pub trait State: proto::TypeState<Binding = CORDIC> {
            const BITS: u8;
        }

        pub struct N0;
        pub struct N1;
        pub struct N2;
        pub struct N3;
        pub struct N4;
        pub struct N5;
        pub struct N6;
        pub struct N7;

        macro_rules! impls {
            ( $( ($NAME:ident, $BITS:expr) $(,)? )+ ) => {
                $(
                    impl proto::TypeState for $NAME {
                        type Binding = CORDIC;

                        #[inline]
                        fn set(rb: &mut Self::Binding) {
                            rb.csr.modify(|_, w| w.scale().bits(<Self as State>::BITS));
                        }
                    }

                    impl State for $NAME {
                        const BITS: u8 = $BITS;
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

    pub trait State: proto::TypeState<Binding = CORDIC> {
        type Args: nargs::State;
        type Results: nres::State;
    }

    // function types with argument count encoded
    pub struct Cos;
    pub struct Sin;
    pub struct SinCos;
    pub struct CosM;
    pub struct SinM;
    pub struct SinCosM;
    pub struct ATan2;
    pub struct Magnitude;
    pub struct ATan2Magnitude;
    pub struct ATan<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }
    pub struct CosH;
    pub struct SinH;
    pub struct SinHCosH;
    pub struct ATanH;
    pub struct Ln<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }
    pub struct Sqrt<Scale: scale::State> {
        _scale: PhantomData<Scale>,
    }

    macro_rules! impls {
        // root / config
        ( $( ($NAME:ident < $SCALE:ty >, $FUNC:ident, nargs::$NARGS:ident, nres::$NRES:ident, start( $($START_PARAM:ident),+ )) $(,)?)+ ) => {
            $(
                impl proto::TypeState for $NAME {
                    type Binding = CORDIC;

                    #[inline]
                    fn set(rb: &mut Self::Binding) {
                        <Self as State>::Args::set(rb);
                        <Self as State>::Results::set(rb);

                        rb.csr.modify(|_, w| w.func().$FUNC());
                    }
                }

                impl State for $NAME {
                    type Args = nargs::$NARGS;
                    type Results = nres::$NRES;
                }

                impls!($NAME, nargs::$NARGS, start( $($START_PARAM),+ ));
                impls!($NAME, nres::$NRES);
            )+
        };

        // impl start for one arg
        ($NAME:ty, nargs::One, start( $PRIMARY:ident )) => {
            // arg_type: Q31
            // nargs: 1
            impl<Res> Cordic<arg_type::Q31, Res, $NAME>
            where
                Res: res_type::State,
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
            impl<Res> Cordic<arg_type::Q15, Res, $NAME>
            where
                Res: res_type::State,
            {
                #[doc = "Start evaluating the configured function"]
                #[doc = "with the provided inputs."]
                #[inline]
                pub fn start(&mut self, $PRIMARY: <arg_type::Q15 as DataType>::Fixed) {
                    self.rb
                        .wdata
                        .write(|w| w.arg().bits($PRIMARY.to_bits() as u16 as _));
                }
            }
        };

        // impl start for two args
        ($NAME:ty, nargs::Two, start( $PRIMARY:ident, $SECONDARY:ident )) => {
            // arg_type: Q31
            // nargs: 2
            impl<Res> Cordic<arg_type::Q31, Res, $NAME>
            where
                Res: res_type::State,
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
            impl<Res> Cordic<arg_type::Q15, Res, $NAME>
            where
                Res: res_type::State,
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
                    let reg = ($PRIMARY.to_bits() as u16 as u32) + (($SECONDARY.to_bits() as u16 as u32) << 16);

                    self.rb.wdata.write(|w| w.arg().bits(reg));
                }
            }
        };

        // impl result for one result
        ($NAME:ty, nres::One) => {
            // res_type: Q31
            // nres: 1
            impl<Arg> Cordic<Arg, res_type::Q31, $NAME>
            where
                Arg: arg_type::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is still ongoing.*"]
                #[inline]
                pub fn result(&mut self) -> <res_type::Q31 as DataType>::Fixed {
                    <res_type::Q31 as DataType>::Fixed::from_bits(
                        self.rb.rdata.read().res().bits() as _
                    )
                }
            }

            // res_type: Q15
            // nres: 1
            impl<Arg> Cordic<Arg, res_type::Q15, $NAME>
            where
                Arg: arg_type::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is still ongoing.*"]
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
            impl<Arg> Cordic<Arg, res_type::Q31, $NAME>
            where
                Arg: arg_type::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is still ongoing.*"]
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
            impl<Arg> Cordic<Arg, res_type::Q15, $NAME>
            where
                Arg: arg_type::State,
            {
                #[doc = "Read the evaluation result."]
                #[doc = "\n*Note: This function locks the core if an evaluation"]
                #[doc = "is still ongoing.*"]
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
                    impl proto::TypeState for $NAME<$SCALE> {
                        type Binding = CORDIC;

                        #[inline]
                        fn set(rb: &mut Self::Binding) {
                            <Self as State>::Args::set(rb);
                            <Self as State>::Results::set(rb);
                            <$SCALE>::set(rb);

                            rb.csr.modify(|_, w| w.func().$FUNC());
                        }
                    }

                    impl State for $NAME<$SCALE> {
                        type Args = nargs::$NARGS;
                        type Results = nres::$NRES;
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

    impl<Arg, Res, Func> Cordic<Arg, Res, Func>
    where
        Arg: arg_type::State,
        Res: res_type::State,
        Func: func::State,
    {
        #[inline]
        pub fn func<NewFunc>(mut self) -> Cordic<Arg, Res, NewFunc>
        where
            NewFunc: func::State,
        {
            NewFunc::set(&mut self.rb);

            // SAFETY: the resource was configured
            // such that it is represented by the new
            // type-states
            unsafe { Cordic::wrap(self.rb) }
        }
    }
}

pub struct Cordic<Arg, Res, Func>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State,
{
    rb: CORDIC,
    _arg_size: PhantomData<Arg>,
    _res_size: PhantomData<Res>,
    _func: PhantomData<Func>,
}

#[repr(u8)]
pub enum Precision {
    P4 = 0x1,
    P8,
    P12,
    P16,
    P20,
    P24,
    P28,
    P32,
    P36,
    P40,
    P44,
    P48,
    P52,
    P56,
    P60,
}

impl<Arg, Res, Func> Cordic<Arg, Res, Func>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State,
{
    /// Wrap the CORDIC resource with the
    /// abstraction layer.
    ///
    /// # Safety
    ///
    /// The resulting type's generic type-states
    /// are inferred by context, *not* the state
    /// of the resource. If the resource state
    /// and abstractor state are conflicting, the
    /// established invariance is broken.
    #[inline]
    const unsafe fn wrap(rb: CORDIC) -> Self {
        Cordic {
            rb,
            _arg_size: PhantomData,
            _res_size: PhantomData,
            _func: PhantomData,
        }
    }

    /// Set the precision for the operation (number of iterations).
    #[inline]
    pub fn precision(self, precision: Precision) -> Self {
        // SAFETY: reserved bit value "0" is not a discriminant
        // of the `Precision` enum.
        self.rb
            .csr
            .modify(|_, w| unsafe { w.precision().bits(precision as u8) });

        self
    }

    #[inline]
    pub fn is_ready(&self) -> bool {
        self.rb.csr.read().rrdy().bit_is_set()
    }
}

/// $RM0440 17.4.1
pub type CordicReset = Cordic<arg_type::Q31, res_type::Q31, func::Cos>;

impl<Arg, Res, Func> proto::IntoReset for Cordic<Arg, Res, Func>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State,
{
    type Reset = CordicReset;

    #[inline]
    fn into_reset(self) -> Self::Reset {
        self.arg_type().res_type().func()
    }
}

impl<Arg, Res, Func> proto::Release for Cordic<Arg, Res, Func>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State,
{
    type Resource = CORDIC;

    #[inline]
    unsafe fn release(self) -> Self::Resource {
        self.rb
    }
}

pub mod events {
    pub struct Ready;
}

impl<Arg, Res, Func> proto::Listen for Cordic<Arg, Res, Func>
where
    Arg: arg_type::State,
    Res: res_type::State,
    Func: func::State,
{
    type Events = events::Ready;

    #[inline]
    fn listen(&mut self, _event: Self::Events) {
        self.rb.csr.modify(|_, w| w.ien().set_bit());
    }

    #[inline]
    fn unlisten(&mut self, _event: Self::Events) {
        self.rb.csr.modify(|_, w| w.ien().clear_bit());
    }
}
