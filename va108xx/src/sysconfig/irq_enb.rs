#[doc = "Register `IRQ_ENB` reader"]
pub type R = crate::R<IrqEnbSpec>;
#[doc = "Register `IRQ_ENB` writer"]
pub type W = crate::W<IrqEnbSpec>;
#[doc = "Field `RAMSBE` reader - RAM Single Bit Interrupt"]
pub type RamsbeR = crate::BitReader;
#[doc = "Field `RAMSBE` writer - RAM Single Bit Interrupt"]
pub type RamsbeW<'a, REG> = crate::BitWriter<'a, REG>;
#[doc = "Field `RAMMBE` reader - RAM Multi Bit Interrupt"]
pub type RammbeR = crate::BitReader;
#[doc = "Field `RAMMBE` writer - RAM Multi Bit Interrupt"]
pub type RammbeW<'a, REG> = crate::BitWriter<'a, REG>;
#[doc = "Field `ROMSBE` reader - ROM Single Bit Interrupt"]
pub type RomsbeR = crate::BitReader;
#[doc = "Field `ROMSBE` writer - ROM Single Bit Interrupt"]
pub type RomsbeW<'a, REG> = crate::BitWriter<'a, REG>;
#[doc = "Field `ROMMBE` reader - ROM Multi Bit Interrupt"]
pub type RommbeR = crate::BitReader;
#[doc = "Field `ROMMBE` writer - ROM Multi Bit Interrupt"]
pub type RommbeW<'a, REG> = crate::BitWriter<'a, REG>;
impl R {
    #[doc = "Bit 0 - RAM Single Bit Interrupt"]
    #[inline(always)]
    pub fn ramsbe(&self) -> RamsbeR {
        RamsbeR::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - RAM Multi Bit Interrupt"]
    #[inline(always)]
    pub fn rammbe(&self) -> RammbeR {
        RammbeR::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - ROM Single Bit Interrupt"]
    #[inline(always)]
    pub fn romsbe(&self) -> RomsbeR {
        RomsbeR::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - ROM Multi Bit Interrupt"]
    #[inline(always)]
    pub fn rommbe(&self) -> RommbeR {
        RommbeR::new(((self.bits >> 3) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - RAM Single Bit Interrupt"]
    #[inline(always)]
    #[must_use]
    pub fn ramsbe(&mut self) -> RamsbeW<IrqEnbSpec> {
        RamsbeW::new(self, 0)
    }
    #[doc = "Bit 1 - RAM Multi Bit Interrupt"]
    #[inline(always)]
    #[must_use]
    pub fn rammbe(&mut self) -> RammbeW<IrqEnbSpec> {
        RammbeW::new(self, 1)
    }
    #[doc = "Bit 2 - ROM Single Bit Interrupt"]
    #[inline(always)]
    #[must_use]
    pub fn romsbe(&mut self) -> RomsbeW<IrqEnbSpec> {
        RomsbeW::new(self, 2)
    }
    #[doc = "Bit 3 - ROM Multi Bit Interrupt"]
    #[inline(always)]
    #[must_use]
    pub fn rommbe(&mut self) -> RommbeW<IrqEnbSpec> {
        RommbeW::new(self, 3)
    }
}
#[doc = "Enable EDAC Error Interrupt Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`irq_enb::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`irq_enb::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IrqEnbSpec;
impl crate::RegisterSpec for IrqEnbSpec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`irq_enb::R`](R) reader structure"]
impl crate::Readable for IrqEnbSpec {}
#[doc = "`write(|w| ..)` method takes [`irq_enb::W`](W) writer structure"]
impl crate::Writable for IrqEnbSpec {
    type Safety = crate::Unsafe;
    const ZERO_TO_MODIFY_FIELDS_BITMAP: u32 = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: u32 = 0;
}
#[doc = "`reset()` method sets IRQ_ENB to value 0"]
impl crate::Resettable for IrqEnbSpec {
    const RESET_VALUE: u32 = 0;
}
