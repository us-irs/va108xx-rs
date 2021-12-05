#[doc = "Register `PERIPHERAL_RESET` reader"]
pub struct R(crate::R<PERIPHERAL_RESET_SPEC>);
impl core::ops::Deref for R {
    type Target = crate::R<PERIPHERAL_RESET_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl From<crate::R<PERIPHERAL_RESET_SPEC>> for R {
    #[inline(always)]
    fn from(reader: crate::R<PERIPHERAL_RESET_SPEC>) -> Self {
        R(reader)
    }
}
#[doc = "Register `PERIPHERAL_RESET` writer"]
pub struct W(crate::W<PERIPHERAL_RESET_SPEC>);
impl core::ops::Deref for W {
    type Target = crate::W<PERIPHERAL_RESET_SPEC>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl core::ops::DerefMut for W {
    #[inline(always)]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
impl From<crate::W<PERIPHERAL_RESET_SPEC>> for W {
    #[inline(always)]
    fn from(writer: crate::W<PERIPHERAL_RESET_SPEC>) -> Self {
        W(writer)
    }
}
#[doc = "Field `PORTA` reader - Reset PORTA"]
pub struct PORTA_R(crate::FieldReader<bool, bool>);
impl PORTA_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        PORTA_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for PORTA_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `PORTA` writer - Reset PORTA"]
pub struct PORTA_W<'a> {
    w: &'a mut W,
}
impl<'a> PORTA_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !0x01) | (value as u32 & 0x01);
        self.w
    }
}
#[doc = "Field `PORTB` reader - Reset PORTB"]
pub struct PORTB_R(crate::FieldReader<bool, bool>);
impl PORTB_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        PORTB_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for PORTB_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `PORTB` writer - Reset PORTB"]
pub struct PORTB_W<'a> {
    w: &'a mut W,
}
impl<'a> PORTB_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 1)) | ((value as u32 & 0x01) << 1);
        self.w
    }
}
#[doc = "Field `SPI_0` reader - Reset SPI\\[0\\]"]
pub struct SPI_0_R(crate::FieldReader<bool, bool>);
impl SPI_0_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        SPI_0_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for SPI_0_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `SPI_0` writer - Reset SPI\\[0\\]"]
pub struct SPI_0_W<'a> {
    w: &'a mut W,
}
impl<'a> SPI_0_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 4)) | ((value as u32 & 0x01) << 4);
        self.w
    }
}
#[doc = "Field `SPI_1` reader - Reset SPI\\[1\\]"]
pub struct SPI_1_R(crate::FieldReader<bool, bool>);
impl SPI_1_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        SPI_1_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for SPI_1_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `SPI_1` writer - Reset SPI\\[1\\]"]
pub struct SPI_1_W<'a> {
    w: &'a mut W,
}
impl<'a> SPI_1_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 5)) | ((value as u32 & 0x01) << 5);
        self.w
    }
}
#[doc = "Field `SPI_2` reader - Reset SPI\\[2\\]"]
pub struct SPI_2_R(crate::FieldReader<bool, bool>);
impl SPI_2_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        SPI_2_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for SPI_2_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `SPI_2` writer - Reset SPI\\[2\\]"]
pub struct SPI_2_W<'a> {
    w: &'a mut W,
}
impl<'a> SPI_2_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 6)) | ((value as u32 & 0x01) << 6);
        self.w
    }
}
#[doc = "Field `UART_0` reader - Reset UART\\[0\\]"]
pub struct UART_0_R(crate::FieldReader<bool, bool>);
impl UART_0_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        UART_0_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for UART_0_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `UART_0` writer - Reset UART\\[0\\]"]
pub struct UART_0_W<'a> {
    w: &'a mut W,
}
impl<'a> UART_0_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 8)) | ((value as u32 & 0x01) << 8);
        self.w
    }
}
#[doc = "Field `UART_1` reader - Reset UART\\[1\\]"]
pub struct UART_1_R(crate::FieldReader<bool, bool>);
impl UART_1_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        UART_1_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for UART_1_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `UART_1` writer - Reset UART\\[1\\]"]
pub struct UART_1_W<'a> {
    w: &'a mut W,
}
impl<'a> UART_1_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 9)) | ((value as u32 & 0x01) << 9);
        self.w
    }
}
#[doc = "Field `I2C_0` reader - Reset I2C\\[0\\]"]
pub struct I2C_0_R(crate::FieldReader<bool, bool>);
impl I2C_0_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        I2C_0_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for I2C_0_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `I2C_0` writer - Reset I2C\\[0\\]"]
pub struct I2C_0_W<'a> {
    w: &'a mut W,
}
impl<'a> I2C_0_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 16)) | ((value as u32 & 0x01) << 16);
        self.w
    }
}
#[doc = "Field `I2C_1` reader - Reset I2C\\[1\\]"]
pub struct I2C_1_R(crate::FieldReader<bool, bool>);
impl I2C_1_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        I2C_1_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for I2C_1_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `I2C_1` writer - Reset I2C\\[1\\]"]
pub struct I2C_1_W<'a> {
    w: &'a mut W,
}
impl<'a> I2C_1_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 17)) | ((value as u32 & 0x01) << 17);
        self.w
    }
}
#[doc = "Field `IRQSEL` reader - Reset IRQ selector"]
pub struct IRQSEL_R(crate::FieldReader<bool, bool>);
impl IRQSEL_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        IRQSEL_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for IRQSEL_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `IRQSEL` writer - Reset IRQ selector"]
pub struct IRQSEL_W<'a> {
    w: &'a mut W,
}
impl<'a> IRQSEL_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 21)) | ((value as u32 & 0x01) << 21);
        self.w
    }
}
#[doc = "Field `IOCONFIG` reader - Reset IO Configuration block"]
pub struct IOCONFIG_R(crate::FieldReader<bool, bool>);
impl IOCONFIG_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        IOCONFIG_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for IOCONFIG_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `IOCONFIG` writer - Reset IO Configuration block"]
pub struct IOCONFIG_W<'a> {
    w: &'a mut W,
}
impl<'a> IOCONFIG_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 22)) | ((value as u32 & 0x01) << 22);
        self.w
    }
}
#[doc = "Field `UTILITY` reader - Reset Utility Block"]
pub struct UTILITY_R(crate::FieldReader<bool, bool>);
impl UTILITY_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        UTILITY_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for UTILITY_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `UTILITY` writer - Reset Utility Block"]
pub struct UTILITY_W<'a> {
    w: &'a mut W,
}
impl<'a> UTILITY_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 23)) | ((value as u32 & 0x01) << 23);
        self.w
    }
}
#[doc = "Field `GPIO` reader - Reset GPIO"]
pub struct GPIO_R(crate::FieldReader<bool, bool>);
impl GPIO_R {
    #[inline(always)]
    pub(crate) fn new(bits: bool) -> Self {
        GPIO_R(crate::FieldReader::new(bits))
    }
}
impl core::ops::Deref for GPIO_R {
    type Target = crate::FieldReader<bool, bool>;
    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
#[doc = "Field `GPIO` writer - Reset GPIO"]
pub struct GPIO_W<'a> {
    w: &'a mut W,
}
impl<'a> GPIO_W<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits = (self.w.bits & !(0x01 << 24)) | ((value as u32 & 0x01) << 24);
        self.w
    }
}
impl R {
    #[doc = "Bit 0 - Reset PORTA"]
    #[inline(always)]
    pub fn porta(&self) -> PORTA_R {
        PORTA_R::new((self.bits & 0x01) != 0)
    }
    #[doc = "Bit 1 - Reset PORTB"]
    #[inline(always)]
    pub fn portb(&self) -> PORTB_R {
        PORTB_R::new(((self.bits >> 1) & 0x01) != 0)
    }
    #[doc = "Bit 4 - Reset SPI\\[0\\]"]
    #[inline(always)]
    pub fn spi_0(&self) -> SPI_0_R {
        SPI_0_R::new(((self.bits >> 4) & 0x01) != 0)
    }
    #[doc = "Bit 5 - Reset SPI\\[1\\]"]
    #[inline(always)]
    pub fn spi_1(&self) -> SPI_1_R {
        SPI_1_R::new(((self.bits >> 5) & 0x01) != 0)
    }
    #[doc = "Bit 6 - Reset SPI\\[2\\]"]
    #[inline(always)]
    pub fn spi_2(&self) -> SPI_2_R {
        SPI_2_R::new(((self.bits >> 6) & 0x01) != 0)
    }
    #[doc = "Bit 8 - Reset UART\\[0\\]"]
    #[inline(always)]
    pub fn uart_0(&self) -> UART_0_R {
        UART_0_R::new(((self.bits >> 8) & 0x01) != 0)
    }
    #[doc = "Bit 9 - Reset UART\\[1\\]"]
    #[inline(always)]
    pub fn uart_1(&self) -> UART_1_R {
        UART_1_R::new(((self.bits >> 9) & 0x01) != 0)
    }
    #[doc = "Bit 16 - Reset I2C\\[0\\]"]
    #[inline(always)]
    pub fn i2c_0(&self) -> I2C_0_R {
        I2C_0_R::new(((self.bits >> 16) & 0x01) != 0)
    }
    #[doc = "Bit 17 - Reset I2C\\[1\\]"]
    #[inline(always)]
    pub fn i2c_1(&self) -> I2C_1_R {
        I2C_1_R::new(((self.bits >> 17) & 0x01) != 0)
    }
    #[doc = "Bit 21 - Reset IRQ selector"]
    #[inline(always)]
    pub fn irqsel(&self) -> IRQSEL_R {
        IRQSEL_R::new(((self.bits >> 21) & 0x01) != 0)
    }
    #[doc = "Bit 22 - Reset IO Configuration block"]
    #[inline(always)]
    pub fn ioconfig(&self) -> IOCONFIG_R {
        IOCONFIG_R::new(((self.bits >> 22) & 0x01) != 0)
    }
    #[doc = "Bit 23 - Reset Utility Block"]
    #[inline(always)]
    pub fn utility(&self) -> UTILITY_R {
        UTILITY_R::new(((self.bits >> 23) & 0x01) != 0)
    }
    #[doc = "Bit 24 - Reset GPIO"]
    #[inline(always)]
    pub fn gpio(&self) -> GPIO_R {
        GPIO_R::new(((self.bits >> 24) & 0x01) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Reset PORTA"]
    #[inline(always)]
    pub fn porta(&mut self) -> PORTA_W {
        PORTA_W { w: self }
    }
    #[doc = "Bit 1 - Reset PORTB"]
    #[inline(always)]
    pub fn portb(&mut self) -> PORTB_W {
        PORTB_W { w: self }
    }
    #[doc = "Bit 4 - Reset SPI\\[0\\]"]
    #[inline(always)]
    pub fn spi_0(&mut self) -> SPI_0_W {
        SPI_0_W { w: self }
    }
    #[doc = "Bit 5 - Reset SPI\\[1\\]"]
    #[inline(always)]
    pub fn spi_1(&mut self) -> SPI_1_W {
        SPI_1_W { w: self }
    }
    #[doc = "Bit 6 - Reset SPI\\[2\\]"]
    #[inline(always)]
    pub fn spi_2(&mut self) -> SPI_2_W {
        SPI_2_W { w: self }
    }
    #[doc = "Bit 8 - Reset UART\\[0\\]"]
    #[inline(always)]
    pub fn uart_0(&mut self) -> UART_0_W {
        UART_0_W { w: self }
    }
    #[doc = "Bit 9 - Reset UART\\[1\\]"]
    #[inline(always)]
    pub fn uart_1(&mut self) -> UART_1_W {
        UART_1_W { w: self }
    }
    #[doc = "Bit 16 - Reset I2C\\[0\\]"]
    #[inline(always)]
    pub fn i2c_0(&mut self) -> I2C_0_W {
        I2C_0_W { w: self }
    }
    #[doc = "Bit 17 - Reset I2C\\[1\\]"]
    #[inline(always)]
    pub fn i2c_1(&mut self) -> I2C_1_W {
        I2C_1_W { w: self }
    }
    #[doc = "Bit 21 - Reset IRQ selector"]
    #[inline(always)]
    pub fn irqsel(&mut self) -> IRQSEL_W {
        IRQSEL_W { w: self }
    }
    #[doc = "Bit 22 - Reset IO Configuration block"]
    #[inline(always)]
    pub fn ioconfig(&mut self) -> IOCONFIG_W {
        IOCONFIG_W { w: self }
    }
    #[doc = "Bit 23 - Reset Utility Block"]
    #[inline(always)]
    pub fn utility(&mut self) -> UTILITY_W {
        UTILITY_W { w: self }
    }
    #[doc = "Bit 24 - Reset GPIO"]
    #[inline(always)]
    pub fn gpio(&mut self) -> GPIO_W {
        GPIO_W { w: self }
    }
    #[doc = "Writes raw bits to the register."]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.0.bits(bits);
        self
    }
}
#[doc = "Peripheral Reset Control\n\nThis register you can [`read`](crate::generic::Reg::read), [`write_with_zero`](crate::generic::Reg::write_with_zero), [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`modify`](crate::generic::Reg::modify). See [API](https://docs.rs/svd2rust/#read--modify--write-api).\n\nFor information about available fields see [peripheral_reset](index.html) module"]
pub struct PERIPHERAL_RESET_SPEC;
impl crate::RegisterSpec for PERIPHERAL_RESET_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [peripheral_reset::R](R) reader structure"]
impl crate::Readable for PERIPHERAL_RESET_SPEC {
    type Reader = R;
}
#[doc = "`write(|w| ..)` method takes [peripheral_reset::W](W) writer structure"]
impl crate::Writable for PERIPHERAL_RESET_SPEC {
    type Writer = W;
}
#[doc = "`reset()` method sets PERIPHERAL_RESET to value 0xffff_ffff"]
impl crate::Resettable for PERIPHERAL_RESET_SPEC {
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        0xffff_ffff
    }
}
