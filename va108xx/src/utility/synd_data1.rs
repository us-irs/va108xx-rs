#[doc = "Register `SYND_DATA1` reader"]
pub type R = crate::R<SyndData1Spec>;
#[doc = "Register `SYND_DATA1` writer"]
pub type W = crate::W<SyndData1Spec>;
impl core::fmt::Debug for R {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{}", self.bits())
    }
}
impl W {}
#[doc = "Synd Data 1 Register\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`synd_data1::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`synd_data1::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct SyndData1Spec;
impl crate::RegisterSpec for SyndData1Spec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`synd_data1::R`](R) reader structure"]
impl crate::Readable for SyndData1Spec {}
#[doc = "`write(|w| ..)` method takes [`synd_data1::W`](W) writer structure"]
impl crate::Writable for SyndData1Spec {
    type Safety = crate::Unsafe;
    const ZERO_TO_MODIFY_FIELDS_BITMAP: u32 = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: u32 = 0;
}
#[doc = "`reset()` method sets SYND_DATA1 to value 0"]
impl crate::Resettable for SyndData1Spec {
    const RESET_VALUE: u32 = 0;
}
