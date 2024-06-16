#[doc = "Register `S0_RXFIFOIRQTRG` reader"]
pub type R = crate::R<S0RxfifoirqtrgSpec>;
#[doc = "Register `S0_RXFIFOIRQTRG` writer"]
pub type W = crate::W<S0RxfifoirqtrgSpec>;
impl core::fmt::Debug for R {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{}", self.bits())
    }
}
impl W {}
#[doc = "Slave Rx FIFO IRQ Trigger Level\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`s0_rxfifoirqtrg::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`s0_rxfifoirqtrg::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct S0RxfifoirqtrgSpec;
impl crate::RegisterSpec for S0RxfifoirqtrgSpec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`s0_rxfifoirqtrg::R`](R) reader structure"]
impl crate::Readable for S0RxfifoirqtrgSpec {}
#[doc = "`write(|w| ..)` method takes [`s0_rxfifoirqtrg::W`](W) writer structure"]
impl crate::Writable for S0RxfifoirqtrgSpec {
    type Safety = crate::Unsafe;
    const ZERO_TO_MODIFY_FIELDS_BITMAP: u32 = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: u32 = 0;
}
#[doc = "`reset()` method sets S0_RXFIFOIRQTRG to value 0"]
impl crate::Resettable for S0RxfifoirqtrgSpec {
    const RESET_VALUE: u32 = 0;
}
